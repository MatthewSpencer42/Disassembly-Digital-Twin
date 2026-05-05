# Device Config Builder — User Guide

> Run with: `python3 -m disassembly_skill.config_builder.config_builder_app`

---

## Overview

The Device Config Builder is a graphical tool for authoring **device configuration YAML files** used by the disassembly skills system. Each config file describes:

- What the device is (class, model, dimensions, material)
- Which components it has (chassis, lid, PCB, etc.)
- Where the screw zones are and what type of screws they contain
- The exact disassembly sequence: step-by-step robot actions with all motion parameters

The LLM master agent reads this YAML at runtime and dispatches the correct skill nodes with the correct parameters.

---

## Layout

```
┌────────────────┬──────────────────────────────┬──────────────────┐
│  SIDEBAR       │  STEP CARDS (scrollable)      │  YAML PREVIEW    │
│                │                               │                  │
│  Device info   │  ┌─ HOLD #1 ─────────────┐   │  Live-updating   │
│  Components    │  │  Secure chassis         │   │  YAML output.    │
│  Screw zones   │  │  [parameters...]        │   │  Copy button     │
│  Add Step btns │  └────────────────────────┘   │  at top.         │
│  Sequence list │  ┌─ UNSCREW #2 ───────────┐   │                  │
│  Warnings      │  │  Remove lid screws      │   │                  │
│                │  │  [parameters...]        │   │                  │
│                │  └────────────────────────┘   │                  │
└────────────────┴──────────────────────────────┴──────────────────┘
```

---

## Step-by-step: creating a new HDD config

### 1. Fill in Device info (sidebar top)

| Field | Example | Notes |
|---|---|---|
| Device Class | `hdd` | Generic class name. Not brand-specific. |
| Model / Label | `generic_hdd` | Used as the filename base. |
| L / W / H mm | `147 / 101.6 / 26.1` | 3.5-inch HDD dimensions. |
| Material | `mixed` | Aluminium lid + steel chassis. |
| Fixturing | `gripper_only` | UF850 grips the device; no external jig. |

Hover the **ⓘ** icon next to each field for a description of the options.

---

### 2. Add components (sidebar → COMPONENTS → `+ Add`)

Add one entry per physical part of the device:

| Label | Type | Removable |
|---|---|---|
| `HDD_Chassis` | chassis | No |
| `Top_Lid` | lid | Yes |
| `PCB_Main` | board | Yes |
| `Platter` | drive | Yes |

You also need to add each screw zone as a component (type = `screw_zone`) so it appears as a target option in unscrew steps. Or just add the zones directly in the next section.

---

### 3. Add screw zones (sidebar → SCREW ZONES → `+ Add`)

| Zone name | Count | Type | Unlocked after |
|---|---|---|---|
| `Lid_Screw_Zone` | 6 | torx | — (first screws) |
| `PCB_Screw_Zone` | 4 | torx | Top_Lid removed |
| `Internal_Screw_Zone` | 4 | torx | PCB_Main removed |

---

### 4. Build the sequence (sidebar → ADD STEP buttons)

Click the coloured buttons to append steps, then fill in the parameters in the card that appears.

#### Typical HDD sequence

| Step | Button | Target | Key parameters |
|---|---|---|---|
| 1 | **HOLD** | `HDD_Chassis` | strategy=`lateral_clamp`, grip=101.6 mm, approach=`+y` |
| 2 | **UNSCREW** | `Lid_Screw_Zone` | 6× torx, max_torque=1.5 Nm |
| 3 | **PICKUP** | `Top_Lid` | parallel grasp, approach=`+z`, reveals=`PCB_Main` |
| 4 | **HOLD** | `HDD_Chassis` | re-grip (pickup released the hold) |
| 5 | **UNSCREW** | `PCB_Screw_Zone` | 4× torx |
| — | **TOOL CHANGE** | — | required_bit=`T8` (if bit differs from above) |
| 6 | **PICKUP** | `PCB_Main` | precision grasp, low force ≤ 40 N |
| 7 | **HOLD** | `HDD_Chassis` | re-grip for flip |
| 8 | **FLIP** | `HDD_Chassis` | rotation_deg=180 |
| 9 | **UNSCREW** | `Internal_Screw_Zone` | 4× torx, higher torque (spindle nut) |
| 10 | **FLIP DROP** | `HDD_Chassis` | dumps platters into bin |

Use **↑ ↓** buttons on each card to reorder steps. Use **×** to delete.

---

## Action types

### HOLD  `(UF850 + RG6 gripper)`
Clamps the device so it stays stable during subsequent screwing or pickup steps.
Must be repeated after any PICKUP step because pickup releases the grip.

| Strategy | When to use |
|---|---|
| `lateral_clamp` | Grip both side faces along the width axis |
| `top_down_clamp` | Press down from above (e.g. flat device on table) |
| `edge_clamp` | Grip a specific edge or lip |
| `fixture_press` | Position without closing fully (device already constrained) |

**Grip width** must match the device dimension along the grip axis. RG6 max = 160 mm.

---

### UNSCREW  `(xArm5 + FT300 force sensor + screwdriver)`
Finds and removes all screws in the target screw zone using a spiral search and torque control.

**Important**: if two consecutive unscrew steps use different bit types, insert a **TOOL CHANGE** step between them.

| Parameter | Typical HDD value |
|---|---|
| screw_type | `torx` |
| screw_count | 4–6 |
| max_torque_nm | 1.5 Nm lid, 2.0 Nm spindle |
| rotation_speed_rpm | 50–60 |

---

### PICKUP  `(UF850 + RG6 gripper)`
Grasps and lifts a component, then moves it to the drop zone.

| Grasp type | Best for |
|---|---|
| `parallel` | Flat, accessible surfaces (lid, panel) |
| `precision` | Small or fragile parts (PCB) |
| `pinch` | Thin edges |
| `wide` | Large span components |

**Drop zone** (drop_x / drop_y / drop_z): position in `base_link` frame where the extracted component is placed. Default `[0.92, -0.36, 1.25]` m.

---

### FLIP  `(UF850)`
Rotates the held chassis 180° to expose the underside.
Requires a HOLD step immediately before it.

---

### FLIP DROP  `(UF850)`
Moves the held chassis to an intermediate waypoint, then inverts it over a bin so loose parts (platters, rings) fall free.

---

### TOOL CHANGE  `(manual)`
A pause step that signals to the operator that the screwdriver bit must be swapped before the next unscrew step.

| Field | Meaning |
|---|---|
| required_bit | The bit code that must be installed, e.g. `T8`, `PH1`, `H2.5` |
| method | `manual` (operator swaps), `automatic_future` (reserved) |
| Operator Notes | Message displayed when robot pauses |

**Bit codes**: T = Torx (T4…T25), PH = Phillips (PH0…PH3), H = Hex/Allen (H1.5…H5), TW = Tri-wing, PL = Pentalobe, SL = Slotted.

---

## Keyboard shortcuts

| Shortcut | Action |
|---|---|
| `Ctrl+S` | Save project (JSON, preserves all data) |
| `Ctrl+E` | Export YAML (clean spec-format file for the robot) |

---

## Saving vs exporting

| Operation | Format | Purpose |
|---|---|---|
| **Save** (Ctrl+S) | `.json` | Round-trip project file. Use for continuing editing later. |
| **Export YAML** (Ctrl+E) | `.yaml` | Clean config for the robot. Validated before export. |

Both dialogs default to the `config/device_configs/` directory.

---

## Info icons (ⓘ)

Every field label and dropdown has a **ⓘ** icon. Hover over it to see:
- What the parameter controls
- Valid range or units
- Hardware constraints (e.g. RG6 force limit)

---

## Warnings panel

The sidebar Warnings section shows:
- **Errors** (red): config is invalid and will likely fail at runtime
- **Warnings** (yellow): unusual values or possible oversights (e.g. sequence ends on an active hold)

Validation runs automatically. You can still export with warnings present.

---

## YAML structure

The exported YAML follows this schema:

```yaml
device:
  class: hdd
  model: generic_hdd
  dimensions_mm: {length: 147.0, width: 101.6, height: 26.1}
  material: mixed
  fixturing: gripper_only

components:
  HDD_Chassis: {label: HDD_Chassis, type: chassis, removable: false}
  Top_Lid:     {label: Top_Lid,     type: lid,     removable: true}

screw_zones:
  Lid_Screw_Zone:
    parent_component: Top_Lid
    screw_count: 6
    screw_type: torx
    depends_on_removal_of: []

disassembly_sequence:
  - step: 1
    action: hold
    label: "Lateral clamp chassis"
    target: HDD_Chassis
    depends_on_steps: []
    reveals: []
    parameters:
      strategy: lateral_clamp
      grip_width_mm: 101.6
      ...

metadata:
  hardware:
    manipulation_arm: uf850_rg6
    tooling_arm: xarm5_ft300_screwdriver_micro_camera
    global_camera: intel_realsense_d455
```

---

## Runtime usage

Point the master agent to your config file via the ROS 2 parameter:

```bash
ros2 launch disassembly_skill disassembly_system.launch.py \
    device_config:=config/device_configs/hdd.yaml
```

Or load it programmatically:

```python
from disassembly_skill.device_config import DeviceConfig
cfg = DeviceConfig.load("config/device_configs/hdd.yaml")
```
