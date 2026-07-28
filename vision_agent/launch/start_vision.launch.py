from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    agent_pkg = FindPackageShare("vision_agent")
    tool_pkg = FindPackageShare("tool_camera_pkg")

    launch_orbbec = LaunchConfiguration("launch_orbbec")
    launch_tool_cam = LaunchConfiguration("launch_tool_cam")
    tool_video_device = LaunchConfiguration("tool_video_device")

    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([agent_pkg, "/launch/orbbec_camera.launch.py"]),
        condition=IfCondition(launch_orbbec),
    )

    tool_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([tool_pkg, "/launch/tool_camera.launch.py"]),
        launch_arguments={
            "video_device": tool_video_device,
        }.items(),
        condition=IfCondition(launch_tool_cam),
    )

    crop_node = Node(
        package="vision_agent",
        executable="crop_camera_streams",
        name="camera_crop_republisher",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("vision_agent"), "config", "camera_crop.yaml"]
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_orbbec",
            default_value="true",
            description="Launch the Orbbec Femto Bolt camera driver",
        ),
        DeclareLaunchArgument(
            "launch_tool_cam",
            default_value="true",
            description="Launch the local tool camera driver",
        ),
        DeclareLaunchArgument(
            "tool_video_device",
            default_value="auto",
            description="Optional tool camera video device override",
        ),

        LogInfo(msg="Starting full vision stack"),
        LogInfo(msg=["  Orbbec launch: ", launch_orbbec]),
        LogInfo(msg=["  Tool camera launch: ", launch_tool_cam]),
        SetEnvironmentVariable("PYTHONNOUSERSITE", "1"),
        SetEnvironmentVariable("NO_ALBUMENTATIONS_UPDATE", "1"),
        SetEnvironmentVariable("TRANSFORMERS_VERBOSITY", "error"),
        SetEnvironmentVariable("QT_LOGGING_RULES", "*.debug=false;qt.qpa.*=false"),
        SetEnvironmentVariable("MPLCONFIGDIR", "/tmp/matplotlib"),

        orbbec_launch,

        TimerAction(
            period=2.0,
            actions=[
                tool_cam_launch,
            ],
        ),

        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg="Starting crop republisher and split vision nodes"),
                crop_node,
                Node(
                    package="vision_agent",
                    executable="start_vision_global",
                    name="vision_global_node",
                    output="screen",
                ),
                Node(
                    package="vision_agent",
                    executable="start_vision_local",
                    name="vision_local_node",
                    output="screen",
                ),
                Node(
                    package="vision_agent",
                    executable="start_vision_classifier",
                    name="vision_classifier_node",
                    output="screen",
                ),
                Node(
                    package="vision_agent",
                    executable="start_vision_dashboard",
                    name="vision_dashboard_node",
                    output="screen",
                ),
            ],
        ),
    ])
