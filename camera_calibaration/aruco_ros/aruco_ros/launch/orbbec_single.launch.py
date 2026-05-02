from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    ArUco single-marker detector wired to the Orbbec Femto Bolt colour stream.

    Designed for eye-on-base hand-eye calibration with easy_handeye2.

    Usage
    -----
        ros2 launch aruco_ros orbbec_single.launch.py \\
            marker_id:=0 \\
            marker_size:=0.097

    Orbbec topic layout (camera_name=camera, default)
        /camera/color/image_raw
        /camera/color/camera_info
    TF frame published by Orbbec driver:
        camera_color_optical_frame
    """

    marker_id_arg = DeclareLaunchArgument(
        'marker_id',
        default_value='0',
        description='ArUco marker ID to track',
    )
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.097',
        description='Physical marker size in metres (outer border). 97 mm → 0.097',
    )
    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame',
        default_value='aruco_marker_frame',
        description='TF frame name that will be published for the detected marker',
    )
    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame',
        default_value='',
        description='Leave empty — pose will be published relative to camera_frame',
    )
    corner_refinement_arg = DeclareLaunchArgument(
        'corner_refinement',
        default_value='LINES',
        choices=['NONE', 'HARRIS', 'LINES', 'SUBPIX'],
        description='Sub-pixel corner refinement method',
    )

    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        parameters=[{
            'image_is_rectified': True,
            'marker_size':        LaunchConfiguration('marker_size'),
            'marker_id':          LaunchConfiguration('marker_id'),
            'reference_frame':    LaunchConfiguration('reference_frame'),
            'camera_frame':       'camera_color_optical_frame',
            'marker_frame':       LaunchConfiguration('marker_frame'),
            'corner_refinement':  LaunchConfiguration('corner_refinement'),
        }],
        remappings=[
            ('/image',       '/camera/color/image_raw'),
            ('/camera_info', '/camera/color/camera_info'),
        ],
    )

    return LaunchDescription([
        marker_id_arg,
        marker_size_arg,
        marker_frame_arg,
        reference_frame_arg,
        corner_refinement_arg,
        aruco_single,
    ])
