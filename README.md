# NR Dual-Arm Workspace

Top-level guide for the reduced ROS 2 workspace in `/home/adip/workspace/disassembly_ws/src/agentic_disassembly`.

## New System Quick Start

For a fresh machine, use this order:

1. Install Docker and `docker-compose`.
2. Clone the `teleoperation` branch onto the machine.

HTTPS:

```bash
git clone --branch teleoperation https://github.com/adipdas11/agentic_disassembly.git
```

SSH:

```bash
git clone --branch teleoperation git@github.com:adipdas11/agentic_disassembly.git
```

3. Change into the cloned repository:

```bash
cd agentic_disassembly
```

4. Build the teleop image once:

```bash
cd /home/adip/workspace/disassembly_ws/src/agentic_disassembly
./setup_teleop_docker.sh
```

5. Start the prepared teleop container:

```bash
cd /home/adip/workspace/disassembly_ws/src/agentic_disassembly
./run_teleop_docker.sh
```

6. Inside the container, launch teleoperation:

```bash
ros2 launch arm_teleop webcam_exotica_teleop.launch.py \
  hardware_type:=real \
  use_rviz:=true \
  enable_uf850:=false \
  enable_xarm5:=true \
  xarm5_hand:=right \
  camera_index:=12
```

Notes:

- `./setup_teleop_docker.sh` is the first-time Docker image build step
- after that, `./run_teleop_docker.sh` is the normal entrypoint
- `./run_teleop_docker.sh` rebuilds the main ROS packages inside the container each time
- if the host camera index differs, check it with `v4l2-ctl --list-devices`
- if the integrated webcam is missing, check it on the host first before troubleshooting Docker

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

## Docker

This branch can be run in Docker for the teleoperation stack.

Build the image:

```bash
cd /home/adip/workspace/disassembly_ws/src/agentic_disassembly
docker build -t agentic-disassembly-teleop:humble .
```

Or with Compose:

```bash
cd /home/adip/workspace/disassembly_ws/src/agentic_disassembly
./setup_teleop_docker.sh
```

Start an interactive shell in the container with X11 access already handled:

```bash
cd /home/adip/workspace/disassembly_ws/src/agentic_disassembly
./run_teleop_docker.sh
```

The script:

- runs `xhost +local:root`
- starts `docker-compose run --rm teleop`
- drops you into `/ws`
- rebuilds `nr_dual_arm_description`, `nr_dual_arm_moveit_config`, and `arm_teleop`
- automatically sources `/opt/ros/humble/setup.bash` and `/ws/install/setup.bash`

If you prefer the manual flow:

```bash
cd /home/adip/workspace/disassembly_ws/src/agentic_disassembly
xhost +local:root
docker-compose run --rm teleop
```

Launch webcam teleop from inside the container:

```bash
ros2 launch arm_teleop webcam_exotica_teleop.launch.py \
  hardware_type:=real \
  use_rviz:=true \
  enable_uf850:=false \
  enable_xarm5:=true \
  xarm5_hand:=right \
  camera_index:=12
```

Notes for Docker teleop:

- `docker-compose.yml` uses `network_mode: host` so the container can reach the robot controllers on the same LAN.
- It uses `privileged: true` so webcam devices under `/dev/video*` are visible without per-device remapping.
- The X11 socket is mounted so RViz and the webcam tracking window can open on the host display.
- The workspace is copied into the image and built during `docker build`.
- `./run_teleop_docker.sh` also rebuilds the main teleop packages inside the container each time, so source edits are picked up automatically.
- If you change Docker dependencies or the Docker config itself, rebuild the image with `docker-compose build teleop`.
- Camera numbering inside Docker may differ from the host; verify with `v4l2-ctl --list-devices` and prefer checking host camera availability first.

## Notes

- `arm_teleop` depends on `nr_dual_arm_moveit_config`.
- `nr_dual_arm_moveit_config` is now the only MoveIt stack kept in this workspace.
- The webcam teleop tracker now supports explicit camera selection and auto-detection.
