from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    agent_pkg = FindPackageShare("vision_agent")

    launch_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([agent_pkg, "/launch/start_vision.launch.py"])
    )

    return LaunchDescription([
        LogInfo(msg="Starting full vision system via start_vision.launch.py"),
        launch_stack,
    ])
