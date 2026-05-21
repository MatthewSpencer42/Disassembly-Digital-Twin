from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch the Orbbec Femto Bolt tuned for the crop+vision pipeline.

    Key changes vs default
    ----------------------
      enable_colored_point_cloud=false  — was burning ~1.5 CPU cores at 30fps
      enable_point_cloud=false          — same
      depth_fps=15                      — lowest supported; keeps native depth load low

    NOTE: Always stop the camera with Ctrl+C (not kill -9).
    If the camera shows "Resource busy" errors after a crash, run:
      echo '2-8' | sudo tee /sys/bus/usb/drivers/usb/unbind && sleep 3 && echo '2-8' | sudo tee /sys/bus/usb/drivers/usb/bind
    """
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare("orbbec_camera"), "/launch/femto_bolt.launch.py"]
            ),
            launch_arguments={
                "color_width":  "3840",
                "color_height": "2160",
                "color_fps":    "15",
                "color_format": "MJPG",
                "color_qos":    "default",

                # Keep depth native. The global node already handles color/depth
                # resolution differences. Keep the driver on its known Femto
                # Bolt enum value; depth_registration=false prevents aligned
                # depth publication.
                "depth_registration":  "false",
                "align_mode":          "SW",
                "align_target_stream": "COLOR",
                "depth_fps":           "15",
                "depth_qos":           "default",

                # The disassembly stack does not consume IR/IMU streams. Keeping
                # them off makes Femto Bolt startup more reliable and avoids the
                # depth engine retry path ("try to disable ir stream and try again").
                "enable_ir":                     "false",
                "enable_ir_auto_exposure":       "false",
                "enable_accel":                  "false",
                "enable_gyro":                   "false",
                "enable_sync_output_accel_gyro": "false",
                "enable_frame_sync":             "false",

                "enable_point_cloud":         "false",
                "enable_colored_point_cloud": "false",
            }.items(),
        )
    ])
