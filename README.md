# Agentic Disassembly Workspace

Top-level operator and developer guide for the ROS 2 workspace in `/home/adip/workspace/disassembly_ws/src/agentic_disassembly`.

This workspace contains the dual-arm MoveIt stack, EXOTica integration, scene description, task-level disassembly skills, tool control, and the vision stack used by the cell.

## Contents

- [Overview](#overview)
- [Workspace Layout](#workspace-layout)
- [Build And Source](#build-and-source)
- [Launch Files](#launch-files)
- [Test Scripts](#test-scripts)
- [Skills And Runtime Nodes](#skills-and-runtime-nodes)
- [How EXOTica Is Implemented](#how-exotica-is-implemented)
- [Operational Notes](#operational-notes)

## Overview

The current validated runtime model is split into separate modes:

- Planning mode:
  - `dual_arm_moveit_config/exotica.launch.py`
  - stable RViz interactive markers
  - default `enable_servo:=false`
  - default `enable_joystick:=false`
- Servo/teleop mode:
  - `dual_arm_moveit_config/servo_teleop.launch.py`
  - intended for manual joystick and Servo control
  - not the recommended runtime for RViz interactive-marker planning
- Skill mode:
  - `disassembly_skill/disassembly_system.launch.py`
  - uses Servo internally for skill behaviors where needed
  - keeps joystick disabled

This split exists because keeping Servo and RViz interactive-marker planning live in the same runtime made the arm markers unstable.

## Workspace Layout

<details>
<summary><strong>Packages and major folders</strong></summary>

- `dual_arm_moveit_config/`
  - main MoveIt, ros2_control, Servo, EXOTica, hardware-bridge, and teleop package
- `disassembly_skill/`
  - task-level skills and the EXOTica test runner
- `dual_arm_scene_description/`
  - dual-arm URDF/Xacro scene and RViz visualization launches
- `tool_controller/`
  - tool commander and tool-side serial interface
- `vision_agent/`
  - runtime perception stack
- `vision_training/`
  - training environment and assets
- `camera_calibaration/`
  - hand-eye and calibration utilities
- `exotica/`
  - local EXOTica source tree
- `bio_ik/`
  - local bio_ik source tree
- `rq_fts_ros2_driver/`
  - FT sensor driver source
- `tool_camera_pkg/`
  - tool-camera side package

</details>

## Build And Source

<details open>
<summary><strong>Standard build workflow</strong></summary>

Run from the workspace root:

```bash
cd /home/adip/workspace/disassembly_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Selective rebuild during iteration:

```bash
cd /home/adip/workspace/disassembly_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dual_arm_moveit_config disassembly_skill
source install/setup.bash
```

Python syntax check for launch and node files:

```bash
cd /home/adip/workspace/disassembly_ws/src/agentic_disassembly
python3 -m compileall dual_arm_moveit_config disassembly_skill
```

</details>

## Launch Files

### Recommended entrypoints

<details open>
<summary><strong>1. Stable planning runtime</strong></summary>

Launch:

```bash
cd /home/adip/workspace/disassembly_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_arm_moveit_config exotica.launch.py hardware_type:=twin
```

Use this for:

- RViz planning
- interactive-marker dragging
- planning and execution validation
- EXOTica test-script runs

Key arguments for `dual_arm_moveit_config/launch/exotica.launch.py`:

| Argument | Default | Meaning |
|---|---:|---|
| `hardware_type` | `fake` | `fake`, `real`, `isaac`, or `twin` |
| `use_sim_time` | `auto` | `isaac` becomes `true`; others default to `false` |
| `use_rviz` | `true` | Launch RViz |
| `enable_servo` | `false` | Start Servo nodes |
| `enable_joystick` | `false` | Start joystick teleop |
| `cleanup_existing` | `true` | Terminate stale stack processes before bringup |

Behavior:

- includes `demo.launch.py`
- starts the EXOTica IK server after a short delay
- normalizes `twin` to `real` for xacro/planner hardware mapping
- keeps RViz planning stable by default by not launching Servo/joystick

</details>

<details>
<summary><strong>2. Dedicated Servo and joystick runtime</strong></summary>

Launch:

```bash
cd /home/adip/workspace/disassembly_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dual_arm_moveit_config servo_teleop.launch.py hardware_type:=real
```

Key arguments for `dual_arm_moveit_config/launch/servo_teleop.launch.py`:

| Argument | Default | Meaning |
|---|---:|---|
| `hardware_type` | `real` | `real`, `fake`, `isaac`, or `twin` |
| `use_sim_time` | `auto` | forwarded to `exotica.launch.py` |
| `cleanup_existing` | `true` | cleanup before bringup |
| `use_rviz` | `false` | RViz is off by default |

Forced behavior:

- `enable_servo:=true`
- `enable_joystick:=true`

Use this for:

- manual Servo jogging
- joystick operation
- dedicated teleop sessions

Do not use this as the main RViz planning session if you need stable arm interactive markers.

</details>

<details>
<summary><strong>3. Full disassembly skill bringup</strong></summary>

Launch:

```bash
cd /home/adip/workspace/disassembly_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch disassembly_skill disassembly_system.launch.py
```

Key arguments for `disassembly_skill/launch/disassembly_system.launch.py`:

| Argument | Default | Meaning |
|---|---:|---|
| `hardware_type` | `real` | forwarded into the MoveIt/EXOTica bringup |

Bringup order:

1. `dual_arm_moveit_config/exotica.launch.py`
2. hand-eye publisher
3. FT300 standalone launch
4. vision system
5. debug feed republisher
6. `rqt_image_view`

Important runtime choice:

- internally starts MoveIt with `enable_servo:=true`
- internally starts MoveIt with `enable_joystick:=false`
- skills can switch into Servo locally without exposing joystick teleop

</details>

### Lower-level launch files

<details>
<summary><strong>`dual_arm_moveit_config/launch/demo.launch.py`</strong></summary>

Purpose:

- low-level MoveIt and ros2_control bringup used by `exotica.launch.py`

Declared arguments:

| Argument | Default |
|---|---:|
| `db` | `false` |
| `debug` | `false` |
| `use_rviz` | `true` |
| `enable_servo` | `false` |
| `enable_joystick` | `false` |
| `cleanup_existing` | `true` |
| `use_sim_time` | `false` |
| `hardware_type` | `fake` |

What it starts:

- `robot_state_publisher`
- `ros2_control_node`
- `move_group`
- controller spawners
- optional Servo nodes
- optional joystick and teleop bridge
- tool commander
- RViz
- hardware bridge or Isaac relay depending on mode

Hardware behavior:

- `fake`: fake hardware path
- `real`: real hardware bridge
- `isaac`: filtered joint states path
- `twin`: real hardware bridge plus Isaac mirror relay

</details>

<details>
<summary><strong>`dual_arm_moveit_config/launch/dual_arm_exotica_stream.launch.py`</strong></summary>

Purpose:

- streaming EXOTica target-following runtime on top of the normal EXOTica stack

Arguments:

| Argument | Default |
|---|---:|
| `hardware_type` | `real` |
| `use_rviz` | `true` |
| `enable_servo` | `false` |
| `enable_joystick` | `false` |
| `cleanup_existing` | `true` |
| `rate_hz` | `60.0` |
| `target_timeout_sec` | `0.25` |
| `target_filter_alpha` | `0.2` |
| `command_filter_alpha` | `0.35` |
| `max_joint_step_rad` | `0.03` |
| `publish_demo_targets` | `false` |

Adds:

- `dual_arm_exotica_stream_controller.py`
- optional `dual_arm_exotica_demo_targets.py`

</details>

<details>
<summary><strong>`dual_arm_moveit_config/launch/move_group.launch.py`</strong></summary>

Purpose:

- standalone `move_group` launch with the dual-arm MoveIt config

Declared argument:

- `hardware_type:=fake`

Notes:

- uses the shared runtime-config helper for topic mapping
- remaps to `filtered_joint_states` only for `isaac`

</details>

<details>
<summary><strong>`dual_arm_moveit_config/launch/moveit_rviz.launch.py`</strong></summary>

Purpose:

- standalone MoveIt RViz session

Declared arguments:

| Argument | Default |
|---|---:|
| `hardware_type` | `fake` |
| `rviz_config` | package default |

Notes:

- includes the RViz plugin preload workaround
- remaps to filtered joint states only for `isaac`

</details>

<details>
<summary><strong>`dual_arm_moveit_config/launch/rsp.launch.py`</strong></summary>

Purpose:

- standalone `robot_state_publisher`

Declared argument:

- `hardware_type:=fake`

</details>

<details>
<summary><strong>`dual_arm_moveit_config/launch/spawn_controllers.launch.py`</strong></summary>

Purpose:

- controller spawner helper for an existing `controller_manager`

</details>

<details>
<summary><strong>`dual_arm_scene_description/launch/display.launch.py` and `display_mimic.launch.py`</strong></summary>

Purpose:

- pure scene/URDF visualization
- `joint_state_publisher_gui` plus RViz

Use these when checking:

- robot geometry
- links and frames
- mimic-joint behavior

</details>

## Test Scripts

### EXOTica test runner

<details open>
<summary><strong>`disassembly_skill.test_exotica_planner`</strong></summary>

Entry point:

```bash
ros2 run disassembly_skill test_exotica_planner --ros-args -p execute:=false -p hardware_type:=twin -p tests:=T1,T4
```

Parameters:

| Parameter | Default | Meaning |
|---|---:|---|
| `execute` | `false` | whether motion should be executed |
| `tests` | `T1,T2,T3,T4,T5` | comma-separated test selection |
| `hardware_type` | `fake` | planner/hardware mode |

Test meanings:

| Test | Arm | Meaning |
|---|---|---|
| `T1` | UF850 | EXOTica IK solve only at hover pose |
| `T2` | UF850 | move to hover pose |
| `T3` | UF850 | descend from hover to work height |
| `T4` | xArm5 | EXOTica IK solve only at hover pose |
| `T5` | xArm5 | move to hover, descend, then run spiral search |

Recommended usage:

```bash
ros2 launch dual_arm_moveit_config exotica.launch.py hardware_type:=twin
ros2 run disassembly_skill test_exotica_planner --ros-args -p execute:=false -p hardware_type:=twin -p tests:=T1,T4
```

Validated in the current stable planning runtime.

</details>

### Package lint tests

<details>
<summary><strong>`disassembly_skill/test/`</strong></summary>

These are package-level lint checks:

- `test_flake8.py`
- `test_pep257.py`
- `test_copyright.py`

They are packaging and style checks, not motion-runtime tests.

</details>

## Skills And Runtime Nodes

<details open>
<summary><strong>Primary skill executables in <code>disassembly_skill</code></strong></summary>

Registered console scripts from `disassembly_skill/setup.py`:

| Executable | Purpose |
|---|---|
| `object_hold_skill` | top-down tactile hold / grasp sequence |
| `object_flip_skill` | flip behavior |
| `object_flip_drop_skill` | flip and drop sequence |
| `object_pickup_skill` | pickup routine |
| `unscrew_skill` | unscrewing routine |
| `test_exotica_planner` | EXOTica validation runner |
| `motion_backend` | backend wrapper entrypoint |
| `debug_feed_republisher` | republishes vision debug feed for viewing |
| `master_agent` | task-level orchestration entrypoint |
| `groq_master_agent` | alternate orchestration entrypoint |

</details>

<details>
<summary><strong>Skill motion model</strong></summary>

- skills use `disassembly_skill/motion_backend.py`
- the backend wraps MoveIt execution and local Servo switching
- trajectory controllers are the default state
- Servo controllers are activated only when a skill needs Servo motion
- after Servo motion, the backend returns to the trajectory-controller path

This is why the skill runtime can use Servo internally while the stable planning runtime keeps Servo off by default.

</details>

## How EXOTica Is Implemented

<details open>
<summary><strong>Architecture</strong></summary>

The workspace uses EXOTica in two layers:

1. Launch layer
   - `dual_arm_moveit_config/launch/exotica.launch.py`
   - wraps the standard MoveIt bringup and starts `exotica_ik_server_node.py`
2. Planner layer
   - `dual_arm_moveit_config/dual_arm_moveit_config/exotica_planner.py`
   - builds EXOTica planners from the same URDF/SRDF used by MoveIt
3. Consumer layer
   - `disassembly_skill/motion_backend.py`
   - skill nodes call into the planner/backend instead of using EXOTica directly

</details>

<details>
<summary><strong>Runtime-config integration</strong></summary>

`dual_arm_moveit_config/runtime_config.py` is the shared source of truth for:

- hardware normalization
- joint command topic selection
- joint state topic selection
- Isaac filtered-joint-state behavior

This was added so that:

- `twin` no longer falls back to Isaac topics
- helper launches and EXOTica use the same hardware mapping
- xacro and planner generation stay consistent

</details>

<details>
<summary><strong>Planner generation path</strong></summary>

Inside `dual_arm_moveit_config/dual_arm_moveit_config/exotica_planner.py`:

- the planner imports `pyexotica`
- the dual-arm URDF xacro is rendered with the selected hardware mapping
- the generated URDF is written to `/tmp`
- the EXOTica XML template is patched to point at the generated URDF and the package SRDF
- the solver is loaded and used to produce a `moveit_msgs/RobotTrajectory`

That means the EXOTica planner is not using a separate hand-maintained robot model. It is built from the same core description files used by the MoveIt stack.

</details>

<details>
<summary><strong>Single-arm IK server path</strong></summary>

`exotica_ik_server_node.py` exposes the single-arm EXOTica planners used by the skills and tests:

- UF850 planner for `uf850_arm`
- xArm5 planner for `xarm5_arm_no_slide`

`test_exotica_planner.py` waits for the `/exotica/ready` signal when available before starting tests.

</details>

## Operational Notes

<details>
<summary><strong>Current validated behavior</strong></summary>

- Stable RViz planning:
  - `ros2 launch dual_arm_moveit_config exotica.launch.py hardware_type:=twin`
- Stable EXOTica test execution:
  - `ros2 run disassembly_skill test_exotica_planner ...`
- Dedicated teleop:
  - `ros2 launch dual_arm_moveit_config servo_teleop.launch.py ...`

</details>

<details>
<summary><strong>Important caveats</strong></summary>

- Do not assume Servo and RViz interactive-marker planning can share the same session reliably.
- `twin` means:
  - real hardware drives the main planning state
  - Isaac mirrors the real robot through the relay path
- `isaac` alone is the only mode that should use the filtered joint-state path by default.

</details>

<details>
<summary><strong>Related package README files</strong></summary>

- [dual_arm_moveit_config/README.md](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/dual_arm_moveit_config/README.md)
- [disassembly_skill/README.md](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/disassembly_skill/README.md)

</details>
