from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dual_arm_launch_dir = Path(get_package_share_directory("nr_dual_arm_moveit_config")) / "launch"

    hardware_type = LaunchConfiguration("hardware_type")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_uf850 = LaunchConfiguration("enable_uf850")
    enable_xarm5 = LaunchConfiguration("enable_xarm5")
    joy_dev = LaunchConfiguration("joy_dev")
    linear_speed = LaunchConfiguration("linear_speed_mps")
    yaw_speed = LaunchConfiguration("yaw_speed_rps")

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        parameters=[{"device_id": joy_dev, "autorepeat_rate": 20.0}],
    )

    teleop_node = Node(
        package="arm_teleop",
        executable="joy_arm_teleop",
        name="joy_arm_teleop",
        output="screen",
        parameters=[
            {
                "hardware_type": hardware_type,
                "use_sim_time": use_sim_time,
                "enable_uf850": enable_uf850,
                "enable_xarm5": enable_xarm5,
                "linear_speed_mps": linear_speed,
                "yaw_speed_rps": yaw_speed,
            }
        ],
    )

    control_panel = Node(
        package="arm_teleop",
        executable="joy_teleop_control_panel",
        name="joy_teleop_control_panel",
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="auto"),
            DeclareLaunchArgument("enable_uf850", default_value="false"),
            DeclareLaunchArgument("enable_xarm5", default_value="false"),
            DeclareLaunchArgument(
                "joy_dev", default_value="0",
                description="joy_node device_id (integer index)",
            ),
            DeclareLaunchArgument(
                "linear_speed_mps", default_value="0.05",
                description="EE translation speed m/s per full stick deflection",
            ),
            DeclareLaunchArgument(
                "yaw_speed_rps", default_value="0.3",
                description="Yaw rate rad/s per full stick deflection",
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
            # The included dual-arm launch runs a startup cleanup pass that
            # terminates stale ROS processes, including joy_node. Delay the
            # local joystick driver until that pass has completed.
            TimerAction(period=2.0, actions=[joy_node]),
            teleop_node,
            control_panel,
        ]
    )
