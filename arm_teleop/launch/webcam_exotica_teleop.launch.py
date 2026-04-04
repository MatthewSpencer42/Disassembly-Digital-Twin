from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("arm_teleop"))
    dual_arm_launch_dir = Path(get_package_share_directory("dual_arm_moveit_config")) / "launch"

    hardware_type = LaunchConfiguration("hardware_type")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_file = LaunchConfiguration("config_file")
    exotica_ready_timeout = LaunchConfiguration("exotica_ready_timeout")

    webcam_tracker = Node(
        package="arm_teleop",
        executable="webcam_hand_tracker",
        name="webcam_hand_tracker",
        output="screen",
        parameters=[config_file],
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
        parameters=[config_file, {"hardware_type": hardware_type, "use_sim_time": use_sim_time}],
    )

    def _start_teleop_after_ready(event, _context):
        if getattr(event, "returncode", 1) == 0:
            return [teleop_node]
        return []

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("exotica_ready_timeout", default_value="90.0"),
            DeclareLaunchArgument(
                "config_file",
                default_value=str(package_share / "config" / "webcam_exotica_teleop.yaml"),
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
            webcam_tracker,
            wait_for_exotica,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wait_for_exotica,
                    on_exit=_start_teleop_after_ready,
                )
            ),
        ]
    )
