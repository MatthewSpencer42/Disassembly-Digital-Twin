# Device Config Builder — Implementation Specification

## 1. Purpose and Context

### 1.1 What This Tool Does

The Device Config Builder is a **standalone desktop GUI application** (Tkinter) that allows a human operator to manually create device-specific YAML configuration files for the dual-arm robotic disassembly pipeline. These config files define **what** the robot needs to do to disassemble a specific electronic device, **in what order**, and **with what parameters** for each action.

### 1.2 Why This Tool Exists

The current disassembly pipeline has all its action parameters (grip widths, approach angles, torque limits, target positions, disassembly sequences) **hardcoded for HDDs** inside individual Python skill files (`object_hold_skill.py`, `unscrew_skill.py`, `object_pickup_skill.py`, etc.). This means the system cannot disassemble any device other than an HDD without rewriting code.

The solution is to **externalise** all device-specific parameters into YAML config files. Each device class (HDD, Mini PC, Laptop) gets its own config file. The skill nodes read parameters from the config at runtime instead of using hardcoded values. This tool builds those config files through a guided visual interface.

### 1.3 Where This Fits in the Architecture

```
[Manual Disassembly Study] 
        ↓
[Device Config Builder (THIS TOOL)] → device_config.yaml
        ↓
[device_config.py middleware] ← loads YAML, provides typed access
        ↓
[Refactored Skill Nodes] ← read parameters from config
        ↓
[MasterAgentNode / LangGraph] ← receives device context for LLM prompt
```

The operator first disassembles the device by hand, noting every step. Then they use this tool to formally encode that knowledge into a structured config. Later (Phase 2, not this tool), an exploratory mode will let the LLM generate configs automatically from vision scans.

### 1.4 What This Tool Does NOT Do

- It does **not** connect to ROS or subscribe to any live sensor topics
- It does **not** control the robot arms or read joint states
- It does **not** run inference on vision models
- It is a purely **offline** configuration authoring tool
- A separate tool (`disassembly_recorder.py`, already built) handles live ROS recording

---

## 2. Core Concepts

### 2.1 Device Info

Every config starts with basic device metadata:

| Field | Type | Example | Purpose |
|---|---|---|---|
| `device_class` | string | `hdd`, `mini_pc`, `laptop` | Identifies which config to load at runtime |
| `device_model` | string | `WD_Blue_1TB`, `Generic_MiniPC` | Specific model variant |
| `length_mm` | float | 147.0 | Physical dimensions for hold strategy selection |
| `width_mm` | float | 101.6 | Used to validate grip width feasibility |
| `height_mm` | float | 26.1 | Affects hover height calculations |
| `material` | string | `aluminium`, `plastic`, `mixed` | Informs force limits |
| `fixturing` | string | `gripper_only`, `passive_jig` | How the device is held on the workspace |
| `notes` | string | Free text | Operator notes about the device |

### 2.2 Components

Components are the parts that make up the device. These map directly to the **Scout agent's detection classes**. The operator defines which components exist in the device and their properties.

For reference, the current HDD Scout model has 14 classes:
```
0: Actuator_Arm        7: PCB_Screw_Zone
1: Connector_Port      8: Platter
2: Exterior_Screw_Zone 9: Platter_Separator
3: HDD_Chassis        10: Platter_Separator_Ring
4: Hole               11: Spindle_Hub
5: Internal_Screw_Zone 12: Top_Lid
6: PCB_Main           13: Voice_Coil_Magnet
```

For a new device (Mini PC, Laptop), the operator would define a different set of components matching whatever classes they train in their new Scout model.

Each component entry has:

| Field | Type | Example | Purpose |
|---|---|---|---|
| `label` | string | `PCB_Main` | Must match the Scout model's class name exactly |
| `type` | string | `board`, `lid`, `screw_zone`, `fastener` | Category for grouping |
| `removable` | bool | `true` | Whether this component gets extracted during disassembly |
| `notes` | string | Free text | Any special notes |

### 2.3 Screw Zones and Precedence Mapping

This is the critical data structure. A **screw zone** is a group of screws that must all be removed before a specific parent component can be extracted. The vision model's labelling convention already encodes this relationship:

| Zone Label (from Vision) | Parent Component | Meaning |
|---|---|---|
| `Exterior_Screw_Zone` | `Top_Lid` | Remove all exterior screws → then Top_Lid can be lifted |
| `PCB_Screw_Zone` | `PCB_Main` | Remove all PCB screws → then PCB can be extracted |
| `Internal_Screw_Zone` | `Platter` | Remove internal screws → then platter assembly is free |

For a laptop, this might look like:

| Zone Label | Parent Component | Meaning |
|---|---|---|
| `Bottom_Panel_Screws` | `Bottom_Panel` | Remove screws → bottom panel can be pried/lifted off |
| `Battery_Screws` | `Battery` | Remove screws → battery can be disconnected and lifted |
| `Heatsink_Screws` | `Heatsink` | Remove screws → heatsink lifts off (watch thermal paste) |
| `Motherboard_Screws` | `Motherboard` | Remove screws → motherboard can be extracted |

The **precedence** comes from the order these zones must be processed. You can't access `Heatsink_Screws` until `Bottom_Panel` is removed. You can't access `Motherboard_Screws` until `Heatsink` is removed (it's covering some of them). The config builder lets the operator define these "depends on" relationships.

Data structure per zone:

| Field | Type | Example |
|---|---|---|
| `zone_name` | string | `PCB_Screw_Zone` |
| `parent_component` | string | `PCB_Main` |
| `screw_count` | int | 4 |
| `screw_type` | string | `phillips`, `torx`, `hex`, `pentalobe` |
| `depends_on_removal_of` | list[string] | `["Top_Lid"]` (can't access PCB screws until lid is off) |

### 2.4 Skills and Their Parameters

The config builder supports 5 skills. Each has a set of configurable parameters that the operator sets per device.

#### 2.4.1 Hold / Stabilise

Performed by the **UF850 + RG6 gripper**. Secures the device so the tooling arm can work on it.

| Parameter | Type | Default | Constraints | Description |
|---|---|---|---|---|
| `strategy` | enum | `lateral_clamp` | `lateral_clamp`, `top_down_clamp`, `edge_clamp`, `fixture_press` | How the gripper approaches and grips |
| `grip_width_mm` | float | 85.0 | 0–160 (RG6 max stroke) | Target grip width |
| `approach_axis` | enum | `+z` | `+x`,`-x`,`+y`,`-y`,`+z`,`-z` | Direction the arm approaches from |
| `gripper_open_deg` | float | 35.0 | 0–46 | Gripper open angle before approach |
| `gripper_close_deg` | float | -35.0 | -46–0 | Gripper close angle for clamping |
| `gripper_close_force_n` | float | 100.0 | 0–120 (RG6 max) | Clamping force |
| `torque_threshold_nm` | float | 3.0 | >0 | Joint torque threshold for contact detection |
| `descent_speed_mps` | float | 0.09 | >0, ≤0.2 | Speed during tactile descent |
| `tilt_deg` | float | 7.0 | 0–45 | Approach tilt angle (0 = straight down) |
| `hover_x_offset_m` | float | -0.01 | — | X offset from detected centroid for hover |
| `hover_y_offset_m` | float | -0.005 | — | Y offset from detected centroid for hover |
| `target_component` | string | — | Must match a component label | Which component to hold |
| `notes` | string | — | — | e.g. "Hold HDD sideways so top is accessible" |

**Hardware warning**: If `grip_width_mm > 160`, the UI must show a warning — exceeds RG6 max stroke.

**Strategy descriptions for the UI tooltips**:
- `lateral_clamp`: Approach from the side, grip the device along its shorter axis. Used for HDDs and small enclosures.
- `top_down_clamp`: Approach from above, grip the device edges. Used for medium-sized enclosures that fit within the 160mm stroke.
- `edge_clamp`: Grip one edge of the device from above, pressing it against the workspace. Used when the device is too wide for full clamping.
- `fixture_press`: Do not grip. Instead, press down on the device to hold it against a passive fixture on the workspace. Used for large devices (laptops) that exceed gripper capacity.

#### 2.4.2 Unscrew

Performed by the **xArm5 + screwdriver + Micro Camera + FT300**. Locates, aligns with, and extracts screws.

| Parameter | Type | Default | Constraints | Description |
|---|---|---|---|---|
| `target_zone` | string | — | Must match a screw zone name | Which screw zone to process |
| `screw_type` | enum | `phillips` | `phillips`,`torx`,`hex`,`flat`,`tri_wing`,`pentalobe` | Fastener head type |
| `screw_count` | int | 1 | ≥1 | Number of screws in this zone to remove |
| `engagement_depth_mm` | float | 3.0 | >0 | How deep the bit needs to enter the screw head |
| `initial_torque_nm` | float | 0.5 | >0 | Starting torque for unscrewing |
| `max_torque_nm` | float | 1.5 | >initial | Maximum torque before declaring screw stuck |
| `rotation_speed_rpm` | int | 60 | >0 | Screwdriver motor speed |
| `force_threshold_n` | float | 5.0 | >0 | FT300 force threshold for contact detection |
| `align_tolerance_px` | float | 4.0 | >0 | Pixel error tolerance for alignment complete |
| `spiral_timeout_s` | float | 15.0 | >0 | Max time for spiral search if screw lost |
| `notes` | string | — | — | e.g. "Torx T8, may be tight on older units" |

**Note**: The unscrew skill always uses the xArm5 with FT300 and Micro Camera. These are not configurable — they are hardware facts.

#### 2.4.3 Pick Up / Extract

Performed by the **UF850 + RG6 gripper**. Grasps a freed component and moves it to a deposit location.

| Parameter | Type | Default | Constraints | Description |
|---|---|---|---|---|
| `target_component` | string | — | Must match a component label | What to pick up |
| `grasp_type` | enum | `parallel` | `parallel`,`pinch`,`wide`,`precision` | Gripper strategy |
| `grip_width_mm` | float | 50.0 | 0–160 | Grip width for this component |
| `gripper_close_force_n` | float | 80.0 | 0–120 | Grip force (lower for delicate parts like PCBs) |
| `approach_axis` | enum | `+z` | `+x`,`-x`,`+y`,`-y`,`+z`,`-z` | Approach direction |
| `lift_height_mm` | float | 50.0 | >0 | How high to lift after grasping |
| `lift_speed_mps` | float | 0.02 | >0 | Lift speed (slow for delicate parts) |
| `approach_x_offset_m` | float | -0.02 | — | X offset from detected position |
| `approach_y_offset_m` | float | 0.019 | — | Y offset from detected position |
| `drop_x` | float | 0.92 | — | Drop-off position X (world frame) |
| `drop_y` | float | -0.36 | — | Drop-off position Y |
| `drop_z` | float | 1.25 | — | Drop-off position Z |
| `notes` | string | — | — | e.g. "Lift PCB gently, check for hidden cables" |

**Hardware warning**: If `grip_width_mm > 160`, warn. If `gripper_close_force_n > 120`, warn.

#### 2.4.4 Flip

Performed by the **UF850 + RG6 gripper**. Rotates the device 180° to expose the opposite face.

| Parameter | Type | Default | Constraints | Description |
|---|---|---|---|---|
| `retract_height_m` | float | 0.10 | >0.05 | How high to lift before rotating |
| `gripper_close_force_n` | float | 100.0 | 0–120 | Grip force during flip (must be firm) |
| `rotation_deg` | int | 180 | 180 | Always 180° |
| `notes` | string | — | — | e.g. "Flip to access bottom screws" |

**Precondition**: The UF850 must already be holding the device (hold-state = true). The UI should warn if a flip step is added without a preceding hold step.

#### 2.4.5 Flip Drop

Performed by the **UF850 + RG6 gripper**. Lifts, travels to an intermediate position, flips to dump loose parts, flips back, returns, and re-grasps.

| Parameter | Type | Default | Constraints | Description |
|---|---|---|---|---|
| `retract_height_m` | float | 0.15 | >0.05 | Lift height before travel |
| `gripper_close_force_n` | float | 100.0 | 0–120 | Grip force |
| `intermediate_x` | float | 0.929872 | — | Flip zone X position (world frame) |
| `intermediate_y` | float | -0.633943 | — | Flip zone Y position |
| `intermediate_z` | float | 1.0977 | — | Flip zone Z position |
| `purpose` | string | `dump_loose_parts` | — | Why this flip-drop is needed |
| `notes` | string | — | — | e.g. "Dumps platters and separator rings" |

**Precondition**: Same as flip — must be holding.

---

## 3. Disassembly Sequence Builder

The core of the config builder is the **sequence editor**. This is where the operator defines the ordered list of steps that make up the full disassembly.

### 3.1 Step Structure

Each step in the sequence has:

| Field | Description |
|---|---|
| `step_number` | Auto-assigned sequential integer (1, 2, 3...) |
| `action` | One of: `hold`, `unscrew`, `pickup`, `flip`, `flip_drop` |
| `label` | Human-readable name, e.g. "Hold chassis", "Remove lid screws" |
| `target` | The component or screw zone this action targets |
| `depends_on` | List of step numbers that must complete before this step |
| `reveals` | List of component labels that become visible after this step completes |
| `parameters` | The skill-specific parameters from Section 2.4 |
| `notes` | Free text for operator notes |

### 3.2 Example HDD Sequence

```
Step 1: hold        → target: HDD_Chassis          depends_on: []
Step 2: unscrew     → target: Exterior_Screw_Zone   depends_on: [1]
Step 3: pickup      → target: Top_Lid               depends_on: [2]    reveals: [PCB_Main, PCB_Screw_Zone]
Step 4: hold        → target: HDD_Chassis          depends_on: [3]
Step 5: unscrew     → target: PCB_Screw_Zone        depends_on: [4]
Step 6: pickup      → target: PCB_Main              depends_on: [5]    reveals: [Platter, Internal_Screw_Zone]
Step 7: hold        → target: HDD_Chassis          depends_on: [6]
Step 8: flip        → target: device                depends_on: [7]
Step 9: unscrew     → target: Internal_Screw_Zone   depends_on: [8]
Step 10: flip_drop  → target: device                depends_on: [9]    reveals: [Actuator_Arm, Voice_Coil_Magnet]
```

### 3.3 Example Laptop Sequence (hypothetical)

```
Step 1: hold        → target: Laptop_Chassis        depends_on: []
Step 2: unscrew     → target: Bottom_Panel_Screws   depends_on: [1]
Step 3: pickup      → target: Bottom_Panel          depends_on: [2]    reveals: [Battery, Battery_Screws, RAM, SSD, Heatsink_Screws, Motherboard_Screws]
Step 4: hold        → target: Laptop_Chassis        depends_on: [3]
Step 5: unscrew     → target: Battery_Screws        depends_on: [4]
Step 6: pickup      → target: Battery               depends_on: [5]
Step 7: pickup      → target: RAM                   depends_on: [4]    (no screws, just pull out)
Step 8: pickup      → target: SSD                   depends_on: [4]    (no screws, just pull out)
Step 9: unscrew     → target: Heatsink_Screws       depends_on: [4]
Step 10: pickup     → target: Heatsink              depends_on: [9]    reveals: [CPU, GPU]
Step 11: unscrew    → target: Motherboard_Screws    depends_on: [10]
Step 12: pickup     → target: Motherboard           depends_on: [11]
```

### 3.4 Validation Rules

The UI should enforce or warn about:

1. **First step must be `hold`** — you cannot do anything without stabilising the device first.
2. **`unscrew` must have a preceding `hold`** — the device must be secured before unscrewing.
3. **`pickup` releases the hold** — after a pickup, the hold-state becomes false. If further work is needed, a new `hold` step must follow.
4. **`flip` and `flip_drop` require active hold** — warn if no preceding hold without an intervening pickup that would release it.
5. **Circular dependencies are forbidden** — a step cannot depend on a step with a higher number.
6. **Grip width ≤ 160mm** — hardware limit of the RG6 gripper.
7. **Grip force ≤ 120N** — hardware limit of the RG6 gripper.
8. **FT300 is on xArm5 only** — any parameter referencing force-torque sensing must be associated with the xArm5.

---

## 4. UI Layout and Design

### 4.1 Framework

**Tkinter** — matches the existing `master_agent.py` GUI style. No additional dependencies needed beyond Python standard library.

### 4.2 Window Layout

The application window should be structured as follows:

```
┌──────────────────────────────────────────────────────────────────────┐
│  Menu Bar: [File: New / Open / Save / Export YAML]  [Help]          │
├──────────────┬───────────────────────────────────────────────────────┤
│              │                                                       │
│   SIDEBAR    │              MAIN EDITOR AREA                         │
│              │                                                       │
│  ┌────────┐  │  Shows the detail editor for whatever is selected     │
│  │ Device │  │  in the sidebar:                                      │
│  │  Info  │  │                                                       │
│  ├────────┤  │  - Device Info form (when "Device" selected)          │
│  │ Compo- │  │  - Component list editor (when "Components" selected) │
│  │ nents  │  │  - Zone mapping editor (when "Zones" selected)        │
│  ├────────┤  │  - Step parameter editor (when a step is selected)    │
│  │ Zones  │  │                                                       │
│  ├────────┤  │                                                       │
│  │Sequence│  │                                                       │
│  │        │  │                                                       │
│  │ Step 1 │  │                                                       │
│  │ Step 2 │  │                                                       │
│  │ Step 3 │  │                                                       │
│  │  ...   │  │                                                       │
│  │        │  │                                                       │
│  │[+Add]  │  │                                                       │
│  └────────┘  │                                                       │
│              │                                                       │
├──────────────┴───────────────────────────────────────────────────────┤
│  Status Bar: warnings count | validation status | file path          │
└──────────────────────────────────────────────────────────────────────┘
```

### 4.3 Sidebar Sections

**Device Info** (clickable, shows device form in main area):
- Just shows the device class and model as a label

**Components** (clickable, shows component list in main area):
- Shows count: "Components (14)"
- Clicking opens the component list editor

**Zones** (clickable, shows zone mapping in main area):
- Shows count: "Zones (3)"
- Clicking opens the zone-to-parent mapping editor

**Sequence** (expandable list):
- Each step shown as a compact row:
  ```
  1. hold       → HDD_Chassis
  2. unscrew    → Exterior_Screw_Zone
  3. pickup     → Top_Lid
  ```
- Clicking a step opens its parameter editor in the main area
- Drag to reorder (or up/down buttons)
- [+ Add Step] button at the bottom opens an action type selector
- Right-click or delete button to remove a step

### 4.4 Main Editor Panels

#### Device Info Panel
- Text fields for class, model, material, notes
- Number fields for dimensions (length, width, height in mm)
- Dropdown for fixturing strategy

#### Components Panel
- Scrollable list/table of components
- Each row: label (text), type (dropdown), removable (checkbox), notes (text)
- [+ Add] button to add new component
- [Delete] button per row
- [Import from class list] button — paste a list of class names (one per line) to bulk-add

#### Zones Panel
- Scrollable list of zone mappings
- Each row: zone_name (text), parent_component (dropdown from components list), screw_count (int), screw_type (dropdown), depends_on (multi-select from components)
- [+ Add Zone] button
- Dropdown options for parent_component should only show components marked as `removable`

#### Step Parameter Panel
- Header showing step number, action type, label (editable)
- Target selector (dropdown — components for hold/pickup, zones for unscrew, fixed for flip/flip_drop)
- Depends-on selector (checkboxes of previous step numbers)
- Reveals list (text field, comma-separated component labels)
- Skill-specific parameter fields from Section 2.4 (laid out as labelled entry fields)
- Hardware warnings displayed in red if constraints violated
- Notes text area

### 4.5 Colour Scheme

Match the existing `master_agent.py` dashboard style:

| Element | Colour | Hex |
|---|---|---|
| Background | Dark grey | `#1a1a2e` |
| Surface/cards | Slightly lighter | `#16213e` |
| Accent (hold) | Blue | `#1E90FF` |
| Accent (unscrew) | Orange/amber | `#FFA500` |
| Accent (pickup) | Green | `#32CD32` |
| Accent (flip) | Purple | `#9370DB` |
| Accent (flip_drop) | Pink | `#FF69B4` |
| Text primary | White | `#FFFFFF` |
| Text secondary | Grey | `#888888` |
| Warning | Yellow | `#FFD700` |
| Error | Red | `#FF4444` |

---

## 5. YAML Output Format

The exported config file must follow this exact schema so that `device_config.py` can load it.

```yaml
# Device Configuration: mini_pc / Generic_MiniPC_01
# Generated by Device Config Builder
# Date: 2026-04-05T14:30:00

device:
  class: mini_pc
  model: Generic_MiniPC_01
  dimensions_mm:
    length: 180
    width: 180
    height: 40
  material: mixed
  fixturing: gripper_only
  notes: "Small form factor PC, all screws on bottom panel"

components:
  MiniPC_Chassis:
    label: MiniPC_Chassis
    type: chassis
    removable: false
    notes: "Main structural body"
  Bottom_Panel:
    label: Bottom_Panel
    type: lid
    removable: true
    notes: "4 Phillips screws, plastic clips on edges"
  SSD:
    label: SSD
    type: storage
    removable: true
    notes: "M.2 2280 form factor"
  RAM:
    label: RAM
    type: memory
    removable: true
    notes: "SO-DIMM, clip-retained"
  Motherboard:
    label: Motherboard
    type: board
    removable: true
  Bottom_Panel_Screws:
    label: Bottom_Panel_Screws
    type: screw_zone
    removable: false
  Motherboard_Screws:
    label: Motherboard_Screws
    type: screw_zone
    removable: false
  SSD_Screw:
    label: SSD_Screw
    type: screw_zone
    removable: false

screw_zones:
  Bottom_Panel_Screws:
    parent_component: Bottom_Panel
    screw_count: 4
    screw_type: phillips
    depends_on_removal_of: []
  SSD_Screw:
    parent_component: SSD
    screw_count: 1
    screw_type: phillips
    depends_on_removal_of:
      - Bottom_Panel
  Motherboard_Screws:
    parent_component: Motherboard
    screw_count: 4
    screw_type: phillips
    depends_on_removal_of:
      - Bottom_Panel

disassembly_sequence:
  - step: 1
    action: hold
    label: "Secure chassis"
    target: MiniPC_Chassis
    depends_on_steps: []
    reveals: []
    parameters:
      strategy: top_down_clamp
      grip_width_mm: 130
      approach_axis: "+z"
      gripper_close_force_n: 100
      torque_threshold_nm: 3.0
      descent_speed_mps: 0.09
      tilt_deg: 0
      hover_x_offset_m: 0.0
      hover_y_offset_m: 0.0
    notes: "Top-down clamp on chassis edges"

  - step: 2
    action: unscrew
    label: "Remove bottom panel screws"
    target: Bottom_Panel_Screws
    depends_on_steps: [1]
    reveals: []
    parameters:
      screw_type: phillips
      screw_count: 4
      engagement_depth_mm: 3.0
      initial_torque_nm: 0.5
      max_torque_nm: 1.5
      rotation_speed_rpm: 60
      force_threshold_n: 5.0
      align_tolerance_px: 4.0
      spiral_timeout_s: 15.0
    notes: ""

  - step: 3
    action: pickup
    label: "Remove bottom panel"
    target: Bottom_Panel
    depends_on_steps: [2]
    reveals:
      - SSD
      - RAM
      - Motherboard
      - SSD_Screw
      - Motherboard_Screws
    parameters:
      grasp_type: parallel
      grip_width_mm: 120
      gripper_close_force_n: 60
      approach_axis: "+z"
      lift_height_mm: 80
      lift_speed_mps: 0.02
      drop_x: 0.92
      drop_y: -0.36
      drop_z: 1.25
    notes: "Panel may have clips - if stuck, may need manual assist"

  - step: 4
    action: hold
    label: "Re-secure chassis"
    target: MiniPC_Chassis
    depends_on_steps: [3]
    reveals: []
    parameters:
      strategy: top_down_clamp
      grip_width_mm: 130
      approach_axis: "+z"
      gripper_close_force_n: 100
      torque_threshold_nm: 3.0
      descent_speed_mps: 0.09
      tilt_deg: 0
    notes: "Re-hold after pickup released grip"

  - step: 5
    action: unscrew
    label: "Remove SSD screw"
    target: SSD_Screw
    depends_on_steps: [4]
    reveals: []
    parameters:
      screw_type: phillips
      screw_count: 1
      engagement_depth_mm: 2.0
      initial_torque_nm: 0.3
      max_torque_nm: 1.0
      rotation_speed_rpm: 60
    notes: "Single M.2 retaining screw"

  - step: 6
    action: pickup
    label: "Extract SSD"
    target: SSD
    depends_on_steps: [5]
    reveals: []
    parameters:
      grasp_type: precision
      grip_width_mm: 25
      gripper_close_force_n: 40
      lift_height_mm: 40
      lift_speed_mps: 0.01
    notes: "M.2 SSD, pull at slight angle"

metadata:
  created: "2026-04-05T14:30:00"
  created_by: device_config_builder
  hardware:
    manipulation_arm: uf850_rg6
    tooling_arm: xarm5_ft300_screwdriver_micro_camera
    global_camera: intel_realsense_d455
  framework_version: "1.0"
  total_steps: 6
```

---

## 6. Implementation Notes for Claude Code

### 6.1 File Structure

```
disassembly_skills/
├── disassembly_skills/
│   ├── config_builder/
│   │   ├── __init__.py
│   │   ├── config_builder_app.py    # Main Tkinter application
│   │   ├── panels/
│   │   │   ├── __init__.py
│   │   │   ├── device_panel.py      # Device info editor
│   │   │   ├── components_panel.py  # Component list editor
│   │   │   ├── zones_panel.py       # Zone mapping editor
│   │   │   ├── step_panel.py        # Step parameter editor
│   │   │   └── sequence_panel.py    # Sequence list in sidebar
│   │   ├── models/
│   │   │   ├── __init__.py
│   │   │   └── config_model.py      # Data model (dataclasses)
│   │   ├── utils/
│   │   │   ├── __init__.py
│   │   │   ├── yaml_export.py       # YAML serialisation
│   │   │   ├── validation.py        # Hardware constraint checks
│   │   │   └── constants.py         # Hardware limits, colour scheme, skill params
│   │   └── styles.py                # Tkinter style constants
│   ├── device_config.py             # Config loader (already built)
│   └── ... (existing skill files)
├── config/
│   └── device_configs/
│       ├── hdd_wd_blue.yaml         # Example HDD config
│       ├── mini_pc_generic.yaml     # Example Mini PC config
│       └── ...
└── setup.py                         # Add entry point: config_builder -> config_builder_app:main
```

### 6.2 Key Implementation Decisions

1. **No external dependencies** beyond Python standard library (Tkinter is included). No PyYAML needed — use the simple YAML serialiser from `disassembly_recorder.py` or write a clean one. If PyYAML is available, use it; otherwise fall back to the built-in serialiser.

2. **Data model uses Python dataclasses** — same pattern as `device_config.py` which already defines `DeviceInfo`, `HoldConfig`, `UnscrewConfig`, etc.

3. **File operations**: Save/Load should use JSON internally (for round-trip fidelity) and export to YAML (for human readability and pipeline consumption). The "Save" function saves a `.json` project file; the "Export" function generates the `.yaml` config.

4. **Undo/Redo** is nice to have but not essential for v1. Focus on getting the core workflow right.

5. **The sequence list in the sidebar is the primary navigation element**. Clicking a step shows its parameters in the main area. This mirrors how the master_agent GUI works — a sidebar with status indicators and a main content area.

### 6.3 Entry Point

```python
# In setup.py entry_points:
'config_builder': 'disassembly_skills.config_builder.config_builder_app:main'

# Launch:
ros2 run disassembly_skills config_builder
# or simply:
python3 -m disassembly_skills.config_builder.config_builder_app
```

### 6.4 Testing the Config

After creating a config, the operator can verify it works by:
1. Loading it with `DeviceConfig.load("path/to/config.yaml")`
2. Checking `config.warnings` for any validation issues
3. Printing `config.to_llm_context()` to see what the LLM planner would receive
4. Running the system in interactive/step-by-step mode with the new config

---

## 7. Future Extensions (Not in v1)

These are documented here for awareness but should NOT be implemented in the first version:

1. **ROS integration**: Subscribe to `/vision/agent_state` to auto-populate the component list from live detections
2. **Config preview**: Show a visual diagram of the precedence graph (which zones depend on which removals)
3. **LLM config generation**: Button that sends the component list to an LLM and asks it to generate a plausible disassembly sequence
4. **Template system**: Start from an existing config (e.g., HDD) and modify it for a similar device
5. **Pry and disconnect skills**: Add these when the hardware and skill code support them
6. **Import from recorder**: Load a config generated by `disassembly_recorder.py` for review and refinement
