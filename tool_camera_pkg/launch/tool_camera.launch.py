from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import glob
import os
from ament_index_python.packages import get_package_share_directory


def _canonical_device(path):
    real_path = os.path.realpath(path)
    if real_path.startswith("/dev/") and os.path.exists(real_path):
        return real_path
    return path


def _find_hd_camera_from_sysfs():
    candidates = []
    for name_file in glob.glob("/sys/class/video4linux/video*/name"):
        try:
            with open(name_file, "r", encoding="utf-8") as f:
                name = f.read().strip().lower()
        except OSError:
            continue
        if "hd camera" not in name:
            continue

        video_name = os.path.basename(os.path.dirname(name_file))
        index_file = os.path.join(os.path.dirname(name_file), "index")
        try:
            with open(index_file, "r", encoding="utf-8") as f:
                index = int(f.read().strip())
        except (OSError, ValueError):
            index = 99

        device = f"/dev/{video_name}"
        if os.path.exists(device):
            candidates.append((index, int(video_name.replace("video", "")), device))

    if not candidates:
        return None
    candidates.sort()
    return candidates[0][2]


def _resolve_tool_camera_device(requested_device):
    requested_device = (requested_device or "").strip()
    if requested_device.lower() not in ("", "auto", "default", "none"):
        return _canonical_device(requested_device)

    stable_patterns = [
        "/dev/v4l/by-path/*usb-0:6.2.3*video-index0",
        "/dev/v4l/by-path/*usb-0:6.2.3*",
        "/dev/v4l/by-id/*HD*Camera*video-index0",
        "/dev/v4l/by-id/*HD*Camera*",
    ]
    for pattern in stable_patterns:
        matches = sorted(glob.glob(pattern))
        if matches:
            return _canonical_device(matches[0])

    sysfs_device = _find_hd_camera_from_sysfs()
    if sysfs_device:
        return sysfs_device

    return "/dev/video4"


def _launch_setup(context, *_args, **_kwargs):
    config_file = os.path.join(
        get_package_share_directory('tool_camera_pkg'),
        'config',
        'tool_camera_info.yaml'
    )
    video_device = _resolve_tool_camera_device(
        LaunchConfiguration('video_device').perform(context).strip()
    )

    return [
        LogInfo(msg=["Tool camera video_device: ", video_device]),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='tool_camera',
            namespace='tool_cam',
            parameters=[{
                'video_device': video_device,
                'framerate': 30.0,
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'yuyv2rgb',     
                'camera_name': 'tool_camera',
                'camera_info_url': 'file://' + config_file
            }]
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='auto',
            description=(
                'Tool camera device. Use auto to detect the stable /dev/v4l '
                'HD Camera symlink, or pass /dev/v4l/by-id/... manually.'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
