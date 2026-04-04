import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('dual_arm_scene_description')
    
    # Path to xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'dual_arm_scene.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'display.rviz')

    # Launch Arguments
    prefix_arg = DeclareLaunchArgument('prefix', default_value='robot_')

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', xacro_file, ' prefix:=', LaunchConfiguration('prefix')])
        }]
    )

    # Joint State Publisher GUI
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        prefix_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])
