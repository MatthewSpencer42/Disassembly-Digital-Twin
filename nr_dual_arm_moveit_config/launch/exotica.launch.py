from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node as LaunchNode
from nr_dual_arm_moveit_config.runtime_config import normalize_xacro_hardware_type


def launch_setup(context, *_args, **_kwargs):
    launch_dir = Path(__file__).resolve().parent
    hardware_type = LaunchConfiguration("hardware_type").perform(context)
    planner_hardware_type = normalize_xacro_hardware_type(hardware_type)
    # Auto-detect sim time: isaac always needs it; real/fake/twin never do.
    # A caller can override by passing use_sim_time:=true explicitly.
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    if use_sim_time_str in ("", "auto"):
        use_sim_time = (hardware_type == "isaac")
    else:
        use_sim_time = use_sim_time_str.lower() in {"true", "1", "yes"}

    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "demo.launch.py")),
        launch_arguments={
            "hardware_type": LaunchConfiguration("hardware_type"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "enable_servo": LaunchConfiguration("enable_servo"),
            "enable_joystick": LaunchConfiguration("enable_joystick"),
            "cleanup_existing": LaunchConfiguration("cleanup_existing"),
            "use_sim_time": "true" if use_sim_time else "false",
        }.items(),
    )

    # Start the EXOTica IK server 3 s after MoveIt
    exotica_node = TimerAction(
        period=3.0,
        actions=[
            LaunchNode(
                package="nr_dual_arm_moveit_config",
                executable="exotica_ik_server_node.py",
                name="exotica_ik_server",
                parameters=[
                    {"hardware_type": planner_hardware_type},
                    {"use_sim_time": use_sim_time},
                ],
                output="screen",
            )
        ],
    )

    return [demo_launch, exotica_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware_type", default_value="fake"),
            # "auto" → infer from hardware_type (isaac→true, others→false).
            # Pass "true"/"false" explicitly to override.
            DeclareLaunchArgument("use_sim_time", default_value="auto"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("enable_servo", default_value="false"),
            DeclareLaunchArgument("enable_joystick", default_value="false"),
            DeclareLaunchArgument("cleanup_existing", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
