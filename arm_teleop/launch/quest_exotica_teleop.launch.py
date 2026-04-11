"""
Teleop launch that expects hand-tracking data from a Meta Quest (or any external
source publishing on the standard /teleop_hand_tracking/{left,right}/* topics).
Identical to webcam_exotica_teleop.launch.py except the webcam_hand_tracker node
is omitted — the hand data is assumed to arrive from an external publisher.

Usage:
  ros2 launch arm_teleop quest_exotica_teleop.launch.py \
    hardware_type:=real \
    use_rviz:=true \
    enable_uf850:=false \
    enable_xarm5:=true \
    xarm5_hand:=right
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("arm_teleop"))
    dual_arm_launch_dir = Path(get_package_share_directory("nr_dual_arm_moveit_config")) / "launch"

    hardware_type = LaunchConfiguration("hardware_type")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = LaunchConfiguration("config_file")
    exotica_ready_timeout = LaunchConfiguration("exotica_ready_timeout")
    enable_uf850 = LaunchConfiguration("enable_uf850")
    enable_xarm5 = LaunchConfiguration("enable_xarm5")
    uf850_hand = LaunchConfiguration("uf850_hand")
    xarm5_hand = LaunchConfiguration("xarm5_hand")
    enable_cameras = LaunchConfiguration("enable_cameras")

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(package_share / "launch" / "realsense.launch.py")),
        condition=IfCondition(enable_cameras),
    )

    wait_for_exotica = Node(
        package="arm_teleop",
        executable="wait_for_exotica_ready",
        name="wait_for_exotica_ready",
        output="screen",
        parameters=[{"timeout_sec": exotica_ready_timeout}],
    )

    teleop_node = Node(
        package="arm_teleop",
        executable="exotica_arm_teleop",
        name="exotica_arm_teleop",
        output="screen",
        parameters=[
            config_file,
            {
                "hardware_type": hardware_type,
                "use_sim_time": use_sim_time,
                "enable_uf850": enable_uf850,
                "enable_xarm5": enable_xarm5,
                "uf850.hand": uf850_hand,
                "xarm5.hand": xarm5_hand,
            },
        ],
    )

    control_panel_node = Node(
        package="arm_teleop",
        executable="teleop_control_panel",
        name="teleop_control_panel",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="auto"),
            DeclareLaunchArgument("exotica_ready_timeout", default_value="240.0"),
            DeclareLaunchArgument("enable_uf850", default_value="true"),
            DeclareLaunchArgument("enable_xarm5", default_value="true"),
            DeclareLaunchArgument("uf850_hand", default_value="right"),
            DeclareLaunchArgument("xarm5_hand", default_value="left"),
            DeclareLaunchArgument("enable_cameras", default_value="false"),
            DeclareLaunchArgument(
                "config_file",
                default_value=str(package_share / "config" / "quest_exotica_teleop.yaml"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(dual_arm_launch_dir / "exotica.launch.py")),
                launch_arguments={
                    "hardware_type": hardware_type,
                    "use_rviz": use_rviz,
                    "use_sim_time": use_sim_time,
                    "enable_servo": "false",
                    "enable_joystick": "false",
                    "cleanup_existing": "true",
                }.items(),
            ),
            realsense_launch,
            # No webcam_hand_tracker — hand data comes from Meta Quest via ROS-TCP-Endpoint
            # Expected topics (same as webcam tracker publishes):
            #   /teleop_hand_tracking/right/wrist  (geometry_msgs/PoseStamped)
            #   /teleop_hand_tracking/left/wrist   (geometry_msgs/PoseStamped)
            #   /teleop_hand_tracking/right/fist   (std_msgs/Bool)
            #   /teleop_hand_tracking/left/fist    (std_msgs/Bool)
            #   /teleop_hand_tracking/right/gripper_aperture  (std_msgs/Float32, 0=closed hand..1=open hand)
            #   /teleop_hand_tracking/left/gripper_aperture   (std_msgs/Float32, 0=closed hand..1=open hand)
            #   /teleop_hand_tracking/right/pinky_pinch  (std_msgs/Bool)
            #   /teleop_hand_tracking/left/pinky_pinch   (std_msgs/Bool)
            wait_for_exotica,
            teleop_node,
            control_panel_node,
        ]
    )
