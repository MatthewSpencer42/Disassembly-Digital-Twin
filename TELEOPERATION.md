# Teleoperation Architecture

Implementation guide for the hand-tracking teleoperation stack in `/home/adip/workspace/disassembly_ws/src/agentic_disassembly`.

## Contents

- [Overview](#overview)
- [Current Runtime](#current-runtime)
- [Key Scripts](#key-scripts)
- [Topic Contract](#topic-contract)
- [Control Flow](#control-flow)
- [Gesture Mapping](#gesture-mapping)
- [Launch Arguments And Tuning](#launch-arguments-and-tuning)
- [Why Each Topic Exists](#why-each-topic-exists)
- [How To Replace Webcam With Meta Quest Unity](#how-to-replace-webcam-with-meta-quest-unity)

## Overview

The teleoperation stack is split into two replaceable halves:

- tracking/input side:
  - produces normalized hand state topics
- robot/control side:
  - consumes those topics
  - solves EXOTica IK
  - streams joint commands to the active controllers

This split is deliberate. It means the tracking backend can change from:

- laptop webcam + MediaPipe
- Meta Quest hand tracking
- Unity-based Quest bridge
- another vision or XR tracker

without changing the robot teleoperation logic, as long as the ROS topic contract stays the same.

## Current Runtime

The current teleop runtime is built on:

- [arm_teleop](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop)
- [nr_dual_arm_moveit_config](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/nr_dual_arm_moveit_config)

Important detail:

- teleop now uses the `nr` MoveIt/EXOTica stack
- active hand-to-arm ownership is configurable at launch
- `uf850_arm` uses `rg6_tcp`
- `xarm5_arm_no_slide` uses `xarm_gripper_tcp`

The runtime launch is:

```bash
cd /home/adip/workspace/disassembly_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch arm_teleop webcam_exotica_teleop.launch.py hardware_type:=real use_rviz:=true
```

Bringup order:

1. start `nr_dual_arm_moveit_config/exotica.launch.py`
2. start webcam hand tracker
3. wait for `/exotica/ready`
4. start the teleop controller

That wait is required so the teleop node does not start sending remote EXOTica IK requests before the server is ready.

## Key Scripts

### [webcam_hand_tracker.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/arm_teleop/webcam_hand_tracker.py)

Purpose:

- capture webcam frames with OpenCV
- detect hands with MediaPipe
- convert landmarks into a normalized wrist pose
- compute gesture booleans
- publish a tracker-independent ROS topic interface

What it publishes:

- wrist pose for each hand
- fist state for each hand
- index-pinch state for each hand
- pinky-pinch state for each hand
- debug text
- UI status feedback by hand side

What it does not do:

- no robot logic
- no IK
- no MoveIt calls
- no controller switching

This is the file to replace if the input source changes from webcam to Quest/Unity.

### [exotica_arm_teleop.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/arm_teleop/exotica_arm_teleop.py)

Purpose:

- consume the normalized hand topics
- toggle each arm teleop on/off
- calibrate hand origin to current TCP origin
- convert hand deltas into target robot TCP deltas
- solve single-arm EXOTica IK
- stream filtered joint targets to arm controllers
- toggle grippers from gesture input

Important implementation details:

- runs under `MultiThreadedExecutor`
- uses `ReentrantCallbackGroup`
- this is required because remote EXOTica IK requests wait for asynchronous responses, and a single-threaded executor caused reply starvation and timeouts

### [hand_math.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/arm_teleop/hand_math.py)

Purpose:

- shared quaternion, vector, and interpolation helpers

This keeps the tracker and teleop nodes simpler and avoids duplicating frame math.

### [wait_for_exotica_ready.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/arm_teleop/wait_for_exotica_ready.py)

Purpose:

- block teleop startup until `/exotica/ready` is seen

This prevents the old startup race where teleop tried to attach to EXOTica too early and then fell back into slower local planner paths.

### [webcam_exotica_teleop.launch.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/launch/webcam_exotica_teleop.launch.py)

Purpose:

- compose the tracker, MoveIt/EXOTica bringup, readiness wait, and teleop node into one operator launch

### [webcam_exotica_teleop.yaml](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/config/webcam_exotica_teleop.yaml)

Purpose:

- tune gesture thresholds
- tune filtering
- tune joint streaming limits
- set workspace bounds
- set per-arm enable defaults

## Topic Contract

### Tracker output topics

These are the normalized topics that the robot-side teleop logic consumes.

| Topic | Type | Meaning |
|---|---|---|
| `/teleop_hand_tracking/right/wrist` | `geometry_msgs/PoseStamped` | normalized right-hand wrist pose |
| `/teleop_hand_tracking/left/wrist` | `geometry_msgs/PoseStamped` | normalized left-hand wrist pose |
| `/teleop_hand_tracking/right/fist` | `std_msgs/Bool` | right-hand fist detection |
| `/teleop_hand_tracking/left/fist` | `std_msgs/Bool` | left-hand fist detection |
| `/teleop_hand_tracking/right/pinch` | `std_msgs/Bool` | right-hand thumb-index pinch |
| `/teleop_hand_tracking/left/pinch` | `std_msgs/Bool` | left-hand thumb-index pinch |
| `/teleop_hand_tracking/right/pinky_pinch` | `std_msgs/Bool` | right-hand thumb-pinky pinch |
| `/teleop_hand_tracking/left/pinky_pinch` | `std_msgs/Bool` | left-hand thumb-pinky pinch |
| `/teleop_hand_tracking/debug` | `std_msgs/String` | tracker-side human-readable debug line |

### Teleop status topics

| Topic | Type | Meaning |
|---|---|---|
| `/teleop_status/right_arm_enabled` | `std_msgs/Bool` | whether the robot currently assigned to the right hand is enabled |
| `/teleop_status/left_arm_enabled` | `std_msgs/Bool` | whether the robot currently assigned to the left hand is enabled |

### Teleop service

| Service | Type | Meaning |
|---|---|---|
| `/exotica_arm_teleop/recalibrate` | `std_srvs/Trigger` | clear stored origins and force recalibration |

### EXOTica readiness topic

| Topic | Type | Meaning |
|---|---|---|
| `/exotica/ready` | `std_msgs/Bool` | EXOTica server is initialized and can accept remote IK requests |

## Control Flow

The current robot-side flow is:

1. the tracker publishes normalized hand pose and gesture topics
2. the teleop node stores the latest hand states with timestamps
3. a fist edge toggles the corresponding arm teleop enabled/disabled
4. when an enabled arm sees valid tracking, joint states, and a TCP transform, teleop calibrates:
   - hand origin pose
   - robot TCP origin pose
   - current seed joints
5. every tick:
   - current hand delta is converted to robot target delta
   - translation is rotated from camera frame into robot `base_link`
   - position is clamped to the configured workspace
   - TCP Z is clamped against hard table-clearance limits
   - target pose is filtered
   - EXOTica solves IK for the arm
   - solved joints are filtered and velocity-limited
   - direct joint streaming publishes to the arm trajectory controller topic
6. a pinky-pinch edge toggles that hand’s gripper open/close

Important separation:

- arm motion uses EXOTica IK and arm controllers
- gripper toggles use the dedicated gripper controller path

## Gesture Mapping

Current mapping:

- hand assigned to `uf850_arm`:
  - fist: toggle `uf850_arm` teleop enabled/disabled
  - pinky pinch: toggle `rg6_gripper` open/close
- hand assigned to `xarm5_arm_no_slide`:
  - fist: toggle `xarm5_arm_no_slide` teleop enabled/disabled
  - pinky pinch: toggle `xarm_gripper` open/close

Current non-use of index pinch:

- thumb-index pinch is still published by the tracker
- the teleop node no longer uses it for enable logic

Why keep publishing it:

- it is still useful as a stable normalized gesture channel
- it can be reused later for another operator function without changing the tracker topic contract

## Launch Arguments And Tuning

### Launch arguments

From [webcam_exotica_teleop.launch.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/launch/webcam_exotica_teleop.launch.py):

| Argument | Default | Meaning |
|---|---:|---|
| `hardware_type` | `real` | `fake`, `real`, `isaac`, or `twin` |
| `use_rviz` | `true` | whether MoveIt RViz is launched |
| `use_sim_time` | `false` | forwarded into the MoveIt stack |
| `exotica_ready_timeout` | `90.0` | seconds to wait for `/exotica/ready` |
| `enable_uf850` | `true` | allow UF850 teleop |
| `enable_xarm5` | `true` | allow xArm5 teleop |
| `uf850_hand` | `right` | assign UF850 to `right` or `left` |
| `xarm5_hand` | `left` | assign xArm5 to `right` or `left` |
| `config_file` | package yaml | teleop/tracker parameter file |

Examples:

Only xArm5 teleop:

```bash
ros2 launch arm_teleop webcam_exotica_teleop.launch.py \
  hardware_type:=fake \
  use_rviz:=true \
  enable_uf850:=false \
  enable_xarm5:=true \
  xarm5_hand:=right
```

Only UF850 teleop:

```bash
ros2 launch arm_teleop webcam_exotica_teleop.launch.py \
  hardware_type:=real \
  enable_uf850:=true \
  enable_xarm5:=false
```

Swap both hands:

```bash
ros2 launch arm_teleop webcam_exotica_teleop.launch.py \
  hardware_type:=fake \
  uf850_hand:=left \
  xarm5_hand:=right
```

Assignment rules:

- if both arms are enabled and both are assigned to the same hand, `xarm5_hand` is forced to the opposite hand
- if one arm is disabled, the enabled arm may use either hand
- the webcam UI follows the resolved mapping and enable flags:
  - disabled robot shows `OFF`
  - enabled robot shows assigned hand and current state

### Important YAML parameters

From [webcam_exotica_teleop.yaml](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/config/webcam_exotica_teleop.yaml):

Tracker-side gesture parameters:

- `pinch_on_threshold`
- `pinch_off_threshold`
- `pinky_pinch_on_threshold`
- `pinky_pinch_off_threshold`
- `fist_on_threshold`
- `fist_off_threshold`

Teleop motion parameters:

- `control_rate_hz`
- `translation_scale`
- `translation_scale_xyz`
- `position_filter_alpha`
- `orientation_filter_alpha`
- `joint_command_gain`
- `max_joint_velocity_rad_s`
- `max_joint_step_rad`
- `max_target_step_m`
- `enable_uf850`
- `enable_xarm5`
- `uf850.hand`
- `xarm5.hand`

Workspace/safety parameters:

- `workspace_min`
- `workspace_max`
- `uf850.min_tcp_z`
- `xarm5.min_tcp_z`

## Why Each Topic Exists

### Wrist pose topics

Used because the robot teleop controller needs a continuous hand pose signal, not raw landmarks.

Why `PoseStamped`:

- gives position and orientation together
- carries a frame id and timestamp
- easy to replace across backends

### Gesture boolean topics

Used because discrete operator state changes should not be inferred from pose every time on the robot side.

Why the tracker publishes booleans directly:

- keeps gesture thresholds on the input side
- avoids duplicating detector logic in every teleop consumer
- makes Unity/Quest replacement easier, because the replacement can publish the same booleans

### Debug topic

Used because hand-tracking failures are hard to debug from robot motion alone. The debug string exposes:

- hand presence
- gesture state
- tracker-side pose summary

### Arm-enabled status topics

Used because the tracker overlay shows whether the robot assigned to each hand is currently enabled. This helps the operator understand whether a gesture edge was actually registered, even when hands are reassigned at launch.

## How To Replace Webcam With Meta Quest Unity

The correct replacement strategy is:

- do not rewrite the robot teleop node first
- replace only the tracker/input side
- keep the ROS topic contract unchanged

### Recommended architecture

Unity/Quest side:

1. read Quest hand tracking in Unity
2. compute a hand pose per hand
3. compute gesture booleans per hand
4. publish the same ROS topics as the webcam tracker

Robot/ROS side:

- keep [exotica_arm_teleop.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/arm_teleop/exotica_arm_teleop.py) unchanged

### Minimum Unity outputs required

Unity should publish:

- `/teleop_hand_tracking/right/wrist`
- `/teleop_hand_tracking/left/wrist`
- `/teleop_hand_tracking/right/fist`
- `/teleop_hand_tracking/left/fist`
- `/teleop_hand_tracking/right/pinky_pinch`
- `/teleop_hand_tracking/left/pinky_pinch`

Optional but recommended:

- `/teleop_hand_tracking/right/pinch`
- `/teleop_hand_tracking/left/pinch`
- `/teleop_hand_tracking/debug`

### Message choices for Unity

Best path:

- use ROS-TCP-Connector or another Unity ROS bridge
- publish the same ROS 2 message types already used now:
  - `geometry_msgs/PoseStamped`
  - `std_msgs/Bool`
  - `std_msgs/String`

### Coordinate-frame work for Quest

The hard part is not ROS publishing. The hard part is frame normalization.

Unity/Quest must provide a hand pose that is equivalent in meaning to the current webcam tracker output:

- a stable hand-centered position
- a stable hand orientation
- handedness-resolved left/right topics

Two valid strategies:

1. publish raw Quest-local hand poses and keep using `camera_to_base_rotation`-style conversion in ROS
2. publish already-normalized teleop poses in Unity, so ROS only consumes them directly

Recommended first step:

- keep the normalization policy in ROS as much as possible
- publish a stable Quest hand pose into the same wrist topics
- then retune only the rotation/scaling parameters

That keeps Unity simpler and preserves most of the tuning path already used for webcam teleop.

### What should not change for Quest

These should stay exactly the same if possible:

- teleop arm enable semantics
- gripper toggle semantics
- EXOTica IK and streaming controller logic
- workspace clamps
- TCP floor safety limits
- per-arm enable launch arguments
- per-arm hand-assignment launch arguments

### If a Quest-specific node is added

The clean structure is:

- `quest_hand_tracker.py` or Unity publisher
- same `/teleop_hand_tracking/...` topics
- same `webcam_exotica_teleop.launch.py` shape, or a parallel `quest_exotica_teleop.launch.py`
- same `enable_*` and `*.hand` launch arguments if you want operator behavior parity with the webcam path

In other words, swap this:

- [webcam_hand_tracker.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/arm_teleop/webcam_hand_tracker.py)

with a Quest provider, but keep this:

- [exotica_arm_teleop.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/arm_teleop/exotica_arm_teleop.py)

## Notes

Current hard TCP floor limits:

- UF850 / `rg6_tcp`: `z >= 0.92962`
- xArm5 / `xarm_gripper_tcp`: `z >= 0.91775`

Current arm-side EXOTica stack:

- [nr_dual_arm_moveit_config](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/nr_dual_arm_moveit_config)

Current key design rule:

- the hand-tracker side is replaceable
- the robot teleop side should remain tracker-agnostic
