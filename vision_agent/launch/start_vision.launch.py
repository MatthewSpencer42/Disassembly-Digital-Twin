from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='Starting Orbbec crop republisher and split vision nodes'),
        Node(
            package='vision_agent',
            executable='crop_camera_streams',
            name='camera_crop_republisher',
            output='screen',
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('vision_agent'), 'config', 'camera_crop.yaml']
                )
            ],
        ),
        Node(
            package='vision_agent',
            executable='start_vision_global',
            name='vision_global_node',
            output='screen'
        ),
        Node(
            package='vision_agent',
            executable='start_vision_local',
            name='vision_local_node',
            output='screen'
        ),
        Node(
            package='vision_agent',
            executable='start_vision_classifier',
            name='vision_classifier_node',
            output='screen'
        ),
        Node(
            package='vision_agent',
            executable='start_vision_dashboard',
            name='vision_dashboard_node',
            output='screen'
        )
    ])
