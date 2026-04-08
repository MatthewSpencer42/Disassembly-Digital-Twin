FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    libmsgpack-dev \
    libompl-dev \
    v4l-utils \
    usbutils \
    iputils-ping \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libgtk-3-0 \
    ros-humble-controller-manager \
    ros-humble-controller-manager-msgs \
    ros-humble-geometric-shapes \
    ros-humble-joint-state-publisher-gui \
    ros-humble-joy \
    ros-humble-moveit \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-msgs \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-servo \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-tf2-ros \
    ros-humble-topic-based-ros2-control \
    ros-humble-xacro \
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
    "numpy<2" \
    "mediapipe==0.10.8" \
    opencv-python \
    pyassimp \
    pymodbus \
    PyYAML \
    scipy \
    transforms3d \
    xarm-python-sdk

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
 && rosdep update

WORKDIR /ws/src
COPY . /ws/src/agentic_disassembly

WORKDIR /ws
RUN source /opt/ros/humble/setup.bash \
 && package_paths="$(colcon list --base-paths src --packages-up-to arm_teleop nr_dual_arm_moveit_config nr_dual_arm_description ros_tcp_endpoint exotica_ik_solver exotica_collision_scene_fcl_latest exotica_core_task_maps --paths-only)" \
 && rosdep install --from-paths ${package_paths} --ignore-src -r -y --rosdistro humble --skip-keys "opencv-python python3-pyassimp ros-humble-ompl ompl pinocchio" \
 && colcon build --packages-up-to arm_teleop nr_dual_arm_moveit_config nr_dual_arm_description ros_tcp_endpoint exotica_ik_solver exotica_collision_scene_fcl_latest exotica_core_task_maps

COPY docker/entrypoint.sh /ros_entrypoint_local.sh
RUN chmod +x /ros_entrypoint_local.sh

ENTRYPOINT ["/ros_entrypoint_local.sh"]
CMD ["bash"]
