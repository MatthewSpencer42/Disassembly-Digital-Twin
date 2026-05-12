from launch import LaunchDescription
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def _log_error(message: str):
    def _callback(context, *args, **kwargs):
        get_logger("disassembly_skill_bringup").error(message)
        return []

    return OpaqueFunction(function=_callback)


def _stage_exit_handlers(process_action, stage_name: str, critical: bool = True):
    def _callback(event, context):
        if event.returncode == 0:
            return []
        actions = [_log_error(f"{stage_name} exited with code {event.returncode}. Check the stage logs above.")]
        if critical:
            actions.append(
                EmitEvent(event=Shutdown(reason=f"{stage_name} failed during bringup"))
            )
        return actions

    return RegisterEventHandler(OnProcessExit(target_action=process_action, on_exit=_callback))


def generate_launch_description():
    hardware_type = LaunchConfiguration("hardware_type")
    enable_servo = LaunchConfiguration("enable_servo")
    handeye_calibration_file = (
        Path(get_package_share_directory("dual_arm_moveit_config")) / "config" / "realsense_handeye.calib"
    )

    moveit_stage = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "dual_arm_moveit_config",
            "exotica.launch.py",
            [TextSubstitution(text="hardware_type:="), hardware_type],
            [TextSubstitution(text="enable_servo:="), enable_servo],
            "enable_joystick:=false",
        ],
        output="screen",
        name="disassembly_moveit_stage",
    )

    handeye_stage = Node(
        package="easy_handeye2",
        executable="handeye_publisher",
        name="realsense_handeye_publisher",
        output="screen",
        parameters=[
            {
                "name": "realsense_handeye",
                "calibration_file": str(handeye_calibration_file),
            }
        ],
    )

    ft_stage = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "robotiq_ft_sensor_hardware",
            "ft_sensor_standalone.launch.py",
            "frame_id:=FT300",
        ],
        output="screen",
        name="disassembly_ft300_stage",
    )

    vision_stage = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "vision_agent",
            "system_startup.launch.py",
            "publish_handeye:=false",
            "tool_video_device:=/dev/video8",
        ],
        output="screen",
        name="disassembly_vision_stage",
    )

    debug_republisher_stage = Node(
        package="disassembly_skill",
        executable="debug_feed_republisher",
        name="debug_feed_republisher",
        output="screen",
    )

    rqt_stage = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "rqt_image_view",
            "rqt_image_view",
            "/vision/debug_feed/compressed",
        ],
        output="screen",
        name="disassembly_debug_view",
    )

    start_ft_after_moveit = RegisterEventHandler(
        OnProcessStart(
            target_action=moveit_stage,
            on_start=[
                LogInfo(msg="✅ MoveIt stage launched. Waiting before starting hand-eye publisher..."),
                TimerAction(
                    period=5.0,
                    actions=[
                        LogInfo(msg="🚀 [2/5] Starting hand-eye calibration publisher..."),
                        handeye_stage,
                    ],
                ),
            ],
        )
    )

    start_ft_after_handeye = RegisterEventHandler(
        OnProcessStart(
            target_action=handeye_stage,
            on_start=[
                LogInfo(msg="✅ Hand-eye publisher launched. Waiting before starting FT300..."),
                TimerAction(
                    period=2.0,
                    actions=[
                        LogInfo(msg="🚀 [3/5] Starting FT300 standalone..."),
                        ft_stage,
                    ],
                ),
            ],
        )
    )

    start_vision_after_ft = RegisterEventHandler(
        OnProcessStart(
            target_action=ft_stage,
            on_start=[
                LogInfo(msg="✅ FT300 stage launched. Waiting before starting vision..."),
                TimerAction(
                    period=3.0,
                    actions=[
                        LogInfo(msg="🚀 [4/5] Starting vision system..."),
                        vision_stage,
                    ],
                ),
            ],
        )
    )

    start_republisher_after_vision = RegisterEventHandler(
        OnProcessStart(
            target_action=vision_stage,
            on_start=[
                LogInfo(msg="✅ Vision stage launched. Waiting before starting debug feed republisher..."),
                TimerAction(
                    period=8.0,
                    actions=[
                        LogInfo(msg="🚀 [5/6] Starting debug feed republisher..."),
                        debug_republisher_stage,
                    ],
                ),
            ],
        )
    )

    start_rqt_after_republisher = RegisterEventHandler(
        OnProcessStart(
            target_action=debug_republisher_stage,
            on_start=[
                LogInfo(msg="✅ Debug feed republisher launched. Waiting before starting debug viewer..."),
                TimerAction(
                    period=2.0,
                    actions=[
                        LogInfo(msg="🚀 [6/6] Starting rqt image view for /vision/debug_feed_view..."),
                        rqt_stage,
                    ],
                ),
            ],
        )
    )

    rqt_started_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=rqt_stage,
            on_start=[
                LogInfo(msg="✅ rqt image view launched for /vision/debug_feed_view."),
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="real"),
            DeclareLaunchArgument(
                "enable_servo",
                default_value="false",
                description=(
                    "Start MoveIt Servo nodes. Keep false unless moveit_servo is "
                    "installed and Servo controllers are intentionally required."
                ),
            ),
            LogInfo(msg="🚀 [1/5] Starting EXOTica MoveIt stack..."),
            moveit_stage,
            start_ft_after_moveit,
            start_ft_after_handeye,
            start_vision_after_ft,
            start_republisher_after_vision,
            start_rqt_after_republisher,
            rqt_started_handler,
            _stage_exit_handlers(moveit_stage, "EXOTica MoveIt stage", critical=True),
            _stage_exit_handlers(handeye_stage, "Hand-eye publisher stage", critical=True),
            _stage_exit_handlers(ft_stage, "FT300 stage", critical=True),
            _stage_exit_handlers(vision_stage, "Vision stage", critical=True),
            _stage_exit_handlers(debug_republisher_stage, "Debug feed republisher stage", critical=True),
        ]
    )
