# disassembly_skill

`disassembly_skill` is the application package that sits on top of the current dual-arm MoveIt stack and the current vision stack.

## What is current

- Package name: `disassembly_skill`
- MoveIt package used underneath: `dual_arm_moveit_config`
- Primary bringup launch:
  - `ros2 launch disassembly_skill disassembly_system.launch.py`
- Default hardware mode in bringup:
  - `hardware_type:=real`

## Bringup sequence

The main system launch is [disassembly_system.launch.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/disassembly_skill/launch/disassembly_system.launch.py). It starts the system in this order:

1. EXOTica + MoveIt from `dual_arm_moveit_config`
2. hand-eye TF publisher using `dual_arm_moveit_config/config/realsense_handeye.calib`
3. FT300 standalone node
4. vision system
5. debug-feed republisher
6. `rqt_image_view`

The debug viewer is opened on:

- `/vision/debug_feed_view`

This topic is republished from:

- `/vision/debug_feed/compressed`

by [debug_feed_republisher.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/disassembly_skill/disassembly_skill/debug_feed_republisher.py).

## Runtime command

```bash
cd /home/adip/workspace/disassembly_ws
source install/setup.bash
ros2 launch disassembly_skill disassembly_system.launch.py
```

Optional fake mode:

```bash
ros2 launch disassembly_skill disassembly_system.launch.py hardware_type:=fake
```

## Motion backend assumptions

The skill package wraps the current MoveIt backend in:

- [motion_backend.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/disassembly_skill/disassembly_skill/motion_backend.py)

The wrapper is aligned to the current robot naming:

- UF arm group: `uf850_arm`
- xArm group: `xarm5_arm_no_slide`
- gripper joint: `rg6_right_drive_joint`
- planning frame: `base_link`
- UF TCP: `rg6_tcp`

## Servo behavior

The current controller model is:

- trajectory controllers are active by default
- Servo controllers are inactive by default
- the system launch explicitly starts the MoveIt stack with:
  - `enable_servo:=true`
  - `enable_joystick:=false`
- the skill backend switches to Servo controllers locally when a Servo descent is requested
- after the Servo motion, the backend returns the system to planning mode

This is required because the arm trajectory controllers and Servo controllers command the same joints and cannot both be active at the same time.

If joystick teleop is needed, use the dedicated Servo/teleop launch in `dual_arm_moveit_config` instead of trying to share the same runtime with RViz interactive planning.

## Vision assumptions

The current vision runtime is the optimized monolithic node in:

- [agent_node_v2.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/vision_agent/vision_agent/agent_node_v2.py)

Current behavior:

- CUDA-backed inference is expected
- raw dashboard publishing is disabled for performance
- the console perf spam is disabled by default
- dashboard viewing in the skill bringup happens through the republished raw topic `/vision/debug_feed_view`

## Notes

- `DISASSEMBLY_SKILLS_REPORT.md` in this package was originally written against the older `dev_ws` package. It should now be treated as historical context plus a migration note, not as the authoritative runtime description.

## Device Config Builder

The package now includes a standalone offline builder for device-specific disassembly YAML files, plus a typed loader middleware for the skill layer.

Added pieces:

- builder entry point:
  - `ros2 run disassembly_skill device_config_builder`
- builder source:
  - [config_builder_app.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/disassembly_skill/disassembly_skill/config_builder/config_builder_app.py)
- typed loader:
  - [device_config.py](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/disassembly_skill/disassembly_skill/device_config.py)
- default and only device config:
  - [hdd.yaml](/home/adip/workspace/disassembly_ws/src/agentic_disassembly/disassembly_skill/config/device_configs/hdd.yaml)

The builder is intentionally offline. It does not subscribe to ROS topics or command hardware. Its role is:

1. describe the device
2. describe removable components and screw zones
3. define a valid disassembly sequence
4. export a YAML file that the skill layer can load later

The loader API is:

```python
from disassembly_skill.device_config import DeviceConfig

config = DeviceConfig.load("/path/to/device.yaml")
print(config.warnings)
print(config.to_llm_context())
```

Implemented validation rules include:

- first step must be `hold`
- `unscrew` requires an active hold
- `pickup` releases the active hold
- `flip` and `flip_drop` require an active hold
- circular dependency rejection for screw-zone removal dependencies
- grip width capped at `160 mm`
- grip force capped at `120 N`
- FT300-only steps must use `xarm5`

Current scope:

- the config system is implemented and used by `master_agent.py` and `groq_master_agent.py`
- those agents pass the loaded device config into the hold, unscrew, pickup, flip, and flip_drop skill nodes
- the single supported device config in this workspace is the HDD flow in `config/device_configs/hdd.yaml`
