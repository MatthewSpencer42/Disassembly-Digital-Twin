from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = Path(__file__).resolve().parent
    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real"),
            DeclareLaunchArgument("use_sim_time", default_value="auto"),
            DeclareLaunchArgument("cleanup_existing", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_dir / "exotica.launch.py")),
                launch_arguments={
                    "hardware_type": LaunchConfiguration("hardware_type"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "cleanup_existing": LaunchConfiguration("cleanup_existing"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "enable_servo": "true",
                    "enable_joystick": "true",
                }.items(),
            ),
        ]
    )
