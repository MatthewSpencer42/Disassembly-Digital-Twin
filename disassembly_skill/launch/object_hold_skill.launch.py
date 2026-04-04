from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="disassembly_skill",
                executable="object_hold_skill",
                name="object_hold_skill_node",
                output="screen",
            )
        ]
    )
