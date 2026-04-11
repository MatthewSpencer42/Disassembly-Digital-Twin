from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    launch_dir = Path(__file__).resolve().parent
    hardware_type = LaunchConfiguration("hardware_type")
    use_rviz = LaunchConfiguration("use_rviz")
    enable_servo = LaunchConfiguration("enable_servo")
    enable_joystick = LaunchConfiguration("enable_joystick")
    cleanup_existing = LaunchConfiguration("cleanup_existing")
    use_sim_time = LaunchConfiguration("use_sim_time")
    publish_demo_targets = LaunchConfiguration("publish_demo_targets")

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="auto"),
            DeclareLaunchArgument("enable_servo", default_value="false"),
            DeclareLaunchArgument("enable_joystick", default_value="false"),
            DeclareLaunchArgument("cleanup_existing", default_value="true"),
            DeclareLaunchArgument("rate_hz", default_value="60.0"),
            DeclareLaunchArgument("target_timeout_sec", default_value="0.25"),
            DeclareLaunchArgument("target_filter_alpha", default_value="0.2"),
            DeclareLaunchArgument("command_filter_alpha", default_value="0.35"),
            DeclareLaunchArgument("max_joint_step_rad", default_value="0.03"),
            DeclareLaunchArgument("publish_demo_targets", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_dir / "exotica.launch.py")),
                launch_arguments={
                    "hardware_type": hardware_type,
                    "use_rviz": use_rviz,
                    "use_sim_time": use_sim_time,
                    "enable_servo": enable_servo,
                    "enable_joystick": enable_joystick,
                    "cleanup_existing": cleanup_existing,
                }.items(),
            ),
            Node(
                package="nr_dual_arm_moveit_config",
                executable="dual_arm_exotica_stream_controller.py",
                name="dual_arm_exotica_stream_controller",
                output="screen",
                parameters=[
                    {
                        "hardware_type": hardware_type,
                        "use_sim_time": use_sim_time,
                        "rate_hz": LaunchConfiguration("rate_hz"),
                        "target_timeout_sec": LaunchConfiguration("target_timeout_sec"),
                        "target_filter_alpha": LaunchConfiguration("target_filter_alpha"),
                        "command_filter_alpha": LaunchConfiguration("command_filter_alpha"),
                        "max_joint_step_rad": LaunchConfiguration("max_joint_step_rad"),
                    }
                ],
            ),
            Node(
                package="nr_dual_arm_moveit_config",
                executable="dual_arm_exotica_demo_targets.py",
                name="dual_arm_exotica_demo_targets",
                output="screen",
                condition=IfCondition(publish_demo_targets),
                parameters=[{"rate_hz": 30.0}],
            ),
        ]
    )
