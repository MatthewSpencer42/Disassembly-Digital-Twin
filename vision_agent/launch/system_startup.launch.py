from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    # --- 1. DEFINE PATHS ---
    rs_pkg = FindPackageShare('realsense2_camera')
    tool_pkg = FindPackageShare('tool_camera_pkg')
    agent_pkg = FindPackageShare('vision_agent')
    publish_handeye = LaunchConfiguration('publish_handeye')
    tool_video_device = LaunchConfiguration('tool_video_device')
    handeye_calibration_file = (
        Path(get_package_share_directory('dual_arm_moveit_config'))
        / 'config'
        / 'realsense_handeye.calib'
    )

    # --- 2. DEFINE LAUNCH ACTIONS ---
    
    # A. RealSense (Global Scout)
    # Keeping the necessary parameters for PointCloud and Depth synchronization
    launch_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rs_pkg, '/launch/rs_launch.py']),
        launch_arguments={
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'pointcloud.stream_filter': '2', # Texture from Color
            'pointcloud.allow_no_texture_points': 'true',
            'pointcloud.ordered_pc': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_sync': 'true',
            'tf_publish_rate': '10.0',
        }.items()
    )

    # B. Tool Camera (Local Sniper)
    launch_tool_cam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([tool_pkg, '/launch/tool_camera.launch.py']),
        launch_arguments={
            'video_device': tool_video_device,
        }.items(),
    )

    launch_handeye = Node(
        package='easy_handeye2',
        executable='handeye_publisher',
        name='realsense_handeye_publisher',
        output='screen',
        parameters=[
            {
                'name': 'realsense_handeye',
                'calibration_file': str(handeye_calibration_file),
            }
        ],
        condition=IfCondition(publish_handeye),
    )

    # C. Vision Agent (Brain)
    launch_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([agent_pkg, '/launch/start_vision.launch.py'])
    )

    # --- 3. CREATE STARTUP SEQUENCE WITH LOGS ---
    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_handeye',
            default_value='true',
            description='Publish dual_arm_moveit_config/config/realsense_handeye.calib',
        ),
        DeclareLaunchArgument(
            'tool_video_device',
            default_value='/dev/video8',
            description='USB video device for the HD tool camera',
        ),
        
        # T+0: Start RealSense
        LogInfo(msg="🚀 [1/4] INITIALIZING GLOBAL SCOUT (REALSENSE)... 📷"),
        launch_realsense,
        launch_handeye,

        # T+3: Start Tool Camera (Wait 3s for RS to settle)
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg="🔧 [2/4] STARTING LOCAL SNIPER (HD TOOL CAM /dev/video8)... 🔬"),
                launch_tool_cam
            ]
        ),

        # T+6: Start AI Agent (Wait 3s for Tool Cam)
        TimerAction(
            period=6.0,
            actions=[
                LogInfo(msg="🧠 [3/4] ACTIVATING VISION AGENT BRAIN... 🤖"),
                launch_agent
            ]
        ),

        # T+8: Final Ready Message
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg="✅ [4/4] VISION READY: REALSENSE CALIBRATION + HD TOOL CAM ONLINE.")
            ]
        )
    ])
