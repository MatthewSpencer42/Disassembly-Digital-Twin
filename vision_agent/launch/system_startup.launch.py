from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    agent_pkg = FindPackageShare("vision_agent")
    tool_video_device = LaunchConfiguration("tool_video_device")

    launch_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([agent_pkg, "/launch/start_vision.launch.py"]),
        launch_arguments={
            "tool_video_device": tool_video_device,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "tool_video_device",
            default_value="auto",
            description="Optional tool camera video device override",
        ),
        LogInfo(msg="Starting full vision system via start_vision.launch.py"),
        launch_stack,
    ])
