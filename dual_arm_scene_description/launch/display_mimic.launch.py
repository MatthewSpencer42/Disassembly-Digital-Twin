import os

from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("dual_arm_scene_description")
    xacro_file = os.path.join(pkg_share, "urdf", "dual_arm_scene_mimic.xacro")
    rviz_config = os.path.join(pkg_share, "rviz", "display.rviz")
    robot_description = Command(["xacro ", xacro_file])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{"robot_description": robot_description, "use_mimic_tags": True}],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            joint_state_publisher,
            rviz,
        ]
    )
