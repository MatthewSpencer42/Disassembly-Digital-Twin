from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    disassembly_system_launch = (
        get_package_share_directory("disassembly_skill")
        + "/launch/disassembly_system.launch.py"
    )

    hardware_type = LaunchConfiguration("hardware_type")
    tool_video_device = LaunchConfiguration("tool_video_device")

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real"),
            DeclareLaunchArgument(
                "tool_video_device",
                default_value="auto",
                description="Optional tool camera video device override",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(disassembly_system_launch),
                launch_arguments={
                    "hardware_type": hardware_type,
                    "tool_video_device": tool_video_device,
                }.items(),
            ),
        ]
    )
