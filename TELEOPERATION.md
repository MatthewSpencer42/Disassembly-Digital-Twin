# Teleoperation Architecture for `agentic_disassembly`

## Current Direction

This workspace now uses a dedicated package, [arm_teleop](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop), for human hand tracking and arm teleoperation.

The current implementation scope is intentionally narrow:

- arm planning only
- no gripper control
- right hand drives `uf850_arm`
- left hand drives `xarm5_arm`
- hand tracking is separated from teleoperation so the tracking backend can be swapped later

This follows the useful part of the Open-Teach design: keep the tracker-specific code separate from the robot-side teleoperation logic. In Open-Teach, that separation appears as a tracker/input path, a robot/operator wrapper, and a robot communication layer. In this workspace the equivalent split is:

- tracking provider: webcam today, Quest later
- normalized hand pose topics: tracker output contract
- teleoperation controller: EXOTica-backed robot-side mapping and command streaming

## Why A Separate Package

The existing teleop code in [teleop_bridge.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/dual_arm_moveit_config/hardware/teleop_bridge.py) is joystick-to-MoveIt-Servo teleoperation. It is not the right place for webcam or VR hand tracking because it is tied to joystick semantics, deadman behavior, and Servo controller switching.

The workspace already had the better building block for this task in [motion_backend.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/dual_arm_moveit_config/dual_arm_moveit_config/motion_backend.py):

- EXOTica single-arm IK support
- direct trajectory-controller joint streaming
- warm-started pose solving
- filtered realtime EXOTica motion logic

So the cleanest design is:

1. keep teleoperation in its own package
2. reuse the existing EXOTica backend from `dual_arm_moveit_config`
3. keep hand tracking replaceable

## Package Layout

The new package is [arm_teleop](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop).

Main files:

- [webcam_hand_tracker.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/arm_teleop/webcam_hand_tracker.py)
- [exotica_arm_teleop.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/arm_teleop/exotica_arm_teleop.py)
- [hand_math.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/arm_teleop/hand_math.py)
- [webcam_exotica_teleop.launch.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/launch/webcam_exotica_teleop.launch.py)
- [webcam_exotica_teleop.yaml](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/config/webcam_exotica_teleop.yaml)

## ROS Graph

### Tracking side

The webcam tracker publishes normalized wrist poses:

- `/teleop_hand_tracking/right/wrist`
- `/teleop_hand_tracking/left/wrist`
- `/teleop_hand_tracking/debug`

The tracker is currently implemented with:

- OpenCV for webcam capture and visualization
- MediaPipe Hands for landmark detection

The visualization window is part of the tracker node. It shows:

- the laptop webcam image
- hand landmarks
- left/right hand labels
- live tracking status

This is the place where a Meta Quest backend can later be swapped in. The teleop node should not care whether the source is:

- laptop webcam + MediaPipe
- Meta Quest hand tracking
- another RGB or depth tracker

as long as the output topics stay the same.

### Teleoperation side

The teleoperation node subscribes only to the wrist pose topics above and uses:

- `right` hand pose -> `uf850_arm`
- `left` hand pose -> `xarm5_arm`

It reuses [MotionBackend](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/dual_arm_moveit_config/dual_arm_moveit_config/motion_backend.py) from `dual_arm_moveit_config` to:

- access current joint states
- access TCP transforms
- reuse EXOTica single-arm pose solving
- stream filtered joint targets through the active trajectory controller

## Control Pipeline

The implemented control path is:

1. webcam tracker estimates hand landmarks
2. wrist, index MCP, and pinky MCP define a stable hand frame
3. tracker publishes `PoseStamped` wrist poses for left and right hands
4. teleop node waits for both hands, joint states, and current robot TCP poses
5. teleop node calibrates by storing:
   - hand origin pose for each hand
   - robot origin TCP pose for each arm
6. runtime motion uses pose deltas:
   - hand translation delta -> robot translation delta
   - hand orientation delta -> robot orientation delta when enabled
7. target pose is filtered and clamped
8. EXOTica solves single-arm pose IK
9. filtered joint commands are streamed to the trajectory controller

## Coordinate Mapping

The webcam tracker produces a normalized hand-centric coordinate system. The teleop node converts that into `base_link` motion using a configurable 3x3 rotation matrix:

- default mapping: camera depth -> robot `x`
- default mapping: image horizontal -> robot `y`
- default mapping: image vertical -> robot `z`

The exact mapping lives in [webcam_exotica_teleop.yaml](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/arm_teleop/config/webcam_exotica_teleop.yaml) under `camera_to_base_rotation`.

This is expected to need tuning on the real setup. That is normal.

## Orientation Policy

The two arms do not currently use the same orientation policy:

- `uf850`: orientation tracking enabled
- `xarm5`: orientation tracking disabled by default

This is intentional. The `xarm5` setup in this workspace is more constrained, so the safer initial behavior is position-driven teleoperation with fixed TCP orientation. That can be relaxed later if testing shows stable pose solving.

## Calibration Behavior

Calibration is automatic.

The teleop node calibrates when all of the following are true:

- right hand is being tracked
- left hand is being tracked
- arm joint states are available
- both current TCP transforms can be read

The teleop node also exposes a recalibration service:

- `/exotica_arm_teleop/recalibrate`

This clears the current origins and asks the operator to hold a new neutral pose.

## Launch

Build:

```bash
cd /home/adip/workspace/disassembly_ws
colcon build --packages-select arm_teleop
source install/setup.bash
```

Run:

```bash
ros2 launch arm_teleop webcam_exotica_teleop.launch.py hardware_type:=real
```

Optional useful overrides:

```bash
ros2 launch arm_teleop webcam_exotica_teleop.launch.py \
  hardware_type:=fake \
  use_rviz:=true
```

The launch file:

- starts the existing MoveIt + EXOTica stack from `dual_arm_moveit_config`
- starts the webcam tracker
- waits for `/exotica/ready`
- only then starts the arm teleoperation node

## Current Limits

Current implementation limits are deliberate:

- no gripper integration
- no pinch-to-action mapping
- no Quest backend yet
- no custom ROS message package yet
- no explicit safety supervisor beyond workspace clamps, target filtering, EXOTica failure rejection, and TCP Z floor limits

## Table Clearance Limits

The teleop path now enforces the same hard lower TCP Z limits that are used in the shared motion backend:

- `uf850` / `rg6_tcp`:
  - `z >= 0.92962`
- `xarm5` / `screwdriver_tcp`:
  - `z >= 0.91775`

These values were taken from the validated table-clearance screenshots and are applied as a lower Z clamp in the teleop workspace, in addition to the existing upper workspace clamp.

The current tracker interface is simple enough that a Quest backend can later publish the same wrist topics and reuse the same teleoperation controller unchanged.

## Next Recommended Steps

1. Tune `camera_to_base_rotation` and `translation_scale` on the real robot.
2. Add a Quest tracking node that publishes the same wrist topics.
3. Add a more explicit `tracking_valid` topic or custom message once the interface stabilizes.
4. Add an enable/disable deadman input for safer real-robot use.
5. Add a TF or RViz visualization of tracked hand frames for easier calibration.

## External References

The design direction here was informed by Open-Teach:

- website: https://open-teach.github.io/
- repository: https://github.com/aadhithya14/Open-Teach
- teleop usage: https://github.com/aadhithya14/Open-Teach/blob/main/docs/teleop_data_collect.md
- extension model: https://github.com/aadhithya14/Open-Teach/blob/main/docs/add_your_own_robot.md
- Quest UI notes: https://github.com/aadhithya14/Open-Teach/blob/main/docs/vr.md

The part reused conceptually is the separation between tracker-specific input handling and robot-specific teleoperation logic. The code in this workspace is not a direct port of Open-Teach; it is a workspace-specific EXOTica-based implementation that fits the existing dual-arm stack.
