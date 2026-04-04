# nr_dual_arm_moveit_config

Current dual-arm MoveIt and Servo integration package for this workspace.

## Main runtime

```bash
cd /home/adip/workspace/disassembly_ws
source install/setup.bash
ros2 launch nr_dual_arm_moveit_config exotica.launch.py hardware_type:=real
```

Current launch defaults are intentionally conservative:

- `enable_servo:=false`
- `enable_joystick:=false`

That keeps RViz planning stable by default. Enable Servo explicitly when needed.

## Runtime split

Use two separate runtime modes:

1. Planning mode
   - RViz + MoveIt planning/execution
   - no Servo nodes
   - stable arm interactive markers
2. Servo/teleop mode
   - MoveIt Servo + joystick teleop
   - intended for direct manual motion, not RViz interactive-marker planning

Planning launch:

```bash
ros2 launch nr_dual_arm_moveit_config exotica.launch.py hardware_type:=real
```

Servo/teleop launch:

```bash
ros2 launch nr_dual_arm_moveit_config servo_teleop.launch.py hardware_type:=real
```

## Current controller model

- planned execution uses:
  - `uf850_controller`
  - `xarm5_controller`
  - `slider_controller`
  - `rg6_controller`
- realtime Servo uses:
  - `uf850_servo_controller`
  - `xarm5_servo_controller`

The arm trajectory controllers and arm Servo controllers cannot be active together on the same arm joints.

## Current joystick behavior

Implemented in [teleop_bridge.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/nr_dual_arm_moveit_config/hardware/teleop_bridge.py):

- default state after startup:
  - trajectory controllers active
- hold `A`:
  - switch to Servo controllers
- release `A`:
  - flush zero twists
  - switch back to trajectory controllers

This behavior is intended for the dedicated Servo/teleop runtime, not for simultaneous RViz interactive-marker planning.

## Current Servo output

The current Servo configs are:

- [xarm_servo.yaml](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/nr_dual_arm_moveit_config/config/xarm_servo.yaml)
- [uf_servo.yaml](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/nr_dual_arm_moveit_config/config/uf_servo.yaml)

Servo publishes `std_msgs/Float64MultiArray` directly to:

- `/xarm5_servo_controller/commands`
- `/uf850_servo_controller/commands`

## Current hand-eye file

- [realsense_handeye.calib](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/nr_dual_arm_moveit_config/config/realsense_handeye.calib)

Frames:

- `base_link`
- `rg6_tcp`
- `camera_link`

## Related docs

- [DUAL_ARM_MOVEIT_CONFIG_FULL_REPORT.md](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/DUAL_ARM_MOVEIT_CONFIG_FULL_REPORT.md)
- [REAL_HARDWARE_SERVO_ROS2_CONTROL_REPORT.md](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/REAL_HARDWARE_SERVO_ROS2_CONTROL_REPORT.md)
