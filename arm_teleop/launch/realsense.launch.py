#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _camera_node(enable_arg: str, name_arg: str, serial_arg: str):
    return Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name="realsense_camera",
        namespace=LaunchConfiguration(name_arg),
        output="screen",
        parameters=[
            {
                "serial_no": LaunchConfiguration(serial_arg),
                "enable_color": True,
                "enable_depth": True,
                "pointcloud.enable": LaunchConfiguration("pointcloud_enable"),
                "align_depth.enable": LaunchConfiguration("align_depth"),
                "rgb_camera.color_profile": "640x480x30",
                "depth_module.depth_profile": "640x480x30",
                "enable_gyro": False,
                "enable_accel": False,
                "enable_infra1": False,
                "enable_infra2": False,
            }
        ],
        condition=IfCondition(LaunchConfiguration(enable_arg)),
    )


def _republisher_node(enable_arg: str, name_arg: str):
    return Node(
        package="image_transport",
        executable="republish",
        name="image_republisher",
        namespace=LaunchConfiguration(name_arg),
        arguments=["raw", "compressed"],
        remappings=[
            ("in", "color/image_raw"),
            ("out/compressed", "color/image_raw/compressed"),
        ],
        parameters=[
            {
                "compressed.jpeg_quality": LaunchConfiguration("jpeg_quality"),
                "compressed.format": "jpeg",
            }
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration(enable_arg)),
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_camera1",
                default_value="true",
                description="Enable camera 1 (D435I)",
            ),
            DeclareLaunchArgument(
                "enable_camera2",
                default_value="true",
                description="Enable camera 2 (D435)",
            ),
            DeclareLaunchArgument(
                "camera1_serial",
                default_value="_233722072289",
                description="Serial number of camera 1 (D435I)",
            ),
            DeclareLaunchArgument(
                "camera2_serial",
                default_value="_231522072957",
                description="Serial number of camera 2 (D435)",
            ),
            DeclareLaunchArgument(
                "camera1_name",
                default_value="camera1",
                description="Namespace for camera 1",
            ),
            DeclareLaunchArgument(
                "camera2_name",
                default_value="camera2",
                description="Namespace for camera 2",
            ),
            DeclareLaunchArgument(
                "pointcloud_enable",
                default_value="false",
                description="Enable point clouds",
            ),
            DeclareLaunchArgument(
                "align_depth",
                default_value="true",
                description="Align depth to color frame",
            ),
            DeclareLaunchArgument(
                "jpeg_quality",
                default_value="80",
                description="JPEG compression quality (1-100)",
            ),
            _camera_node("enable_camera1", "camera1_name", "camera1_serial"),
            _republisher_node("enable_camera1", "camera1_name"),
            _camera_node("enable_camera2", "camera2_name", "camera2_serial"),
            _republisher_node("enable_camera2", "camera2_name"),
        ]
    )
