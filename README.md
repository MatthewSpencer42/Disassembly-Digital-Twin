# NR Dual-Arm Workspace

Top-level guide for the reduced ROS 2 workspace in `/home/adip/workspace/disassembly_ws/src/agentic_disassembly`.

## Remaining packages

- `nr_dual_arm_description`
- `nr_dual_arm_moveit_config`
- `arm_teleop`
- `camera_calibaration`
- `exotica`
- `rq_fts_ros2_driver`

## Build

From the workspace root:

```bash
cd /home/adip/workspace/disassembly_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Selective rebuild:

```bash
cd /home/adip/workspace/disassembly_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nr_dual_arm_description nr_dual_arm_moveit_config arm_teleop
source install/setup.bash
```

## Main runtime entrypoints

NR MoveIt/EXOTica:

```bash
ros2 launch nr_dual_arm_moveit_config exotica.launch.py hardware_type:=real
```

NR demo bringup:

```bash
ros2 launch nr_dual_arm_moveit_config demo.launch.py hardware_type:=real
```

Webcam teleoperation:

```bash
ros2 launch arm_teleop webcam_exotica_teleop.launch.py hardware_type:=real use_rviz:=true
```

Example xArm-only teleop:

```bash
ros2 launch arm_teleop webcam_exotica_teleop.launch.py \
  hardware_type:=real \
  use_rviz:=true \
  enable_uf850:=false \
  enable_xarm5:=true \
  xarm5_hand:=right \
  camera_index:=12
```

Supported teleop launch arguments:

- `hardware_type:=fake|real|isaac|twin`
- `use_rviz:=true|false`
- `use_sim_time:=true|false`
- `enable_uf850:=true|false`
- `enable_xarm5:=true|false`
- `uf850_hand:=right|left`
- `xarm5_hand:=right|left`
- `camera_index:=-1|<video index>`

`camera_index:=-1` means auto-detect the first usable `/dev/video*` device.

## Documentation

- [TELEOPERATION.md](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/TELEOPERATION.md)
- [nr_dual_arm_moveit_config/README.md](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/nr_dual_arm_moveit_config/README.md)
- [nr_dual_arm_description](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/nr_dual_arm_description)

## Notes

- `arm_teleop` depends on `nr_dual_arm_moveit_config`.
- `nr_dual_arm_moveit_config` is now the only MoveIt stack kept in this workspace.
- The webcam teleop tracker now supports explicit camera selection and auto-detection.
