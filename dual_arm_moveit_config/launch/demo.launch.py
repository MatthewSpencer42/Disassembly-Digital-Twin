import os
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from dual_arm_moveit_config.runtime_config import (
    joint_topics_for_hardware,
    normalize_xacro_hardware_type,
    use_filtered_joint_states,
)

# Preload librviz_default_plugins into the global symbol namespace before any
# subprocess is spawned.  class_loader dlopen()s this library as a plugin,
# which would otherwise register a second InteractiveMarkerDisplay factory and
# overwrite the first, leaving a dangling pointer → SIGSEGV on marker drag.
# Setting it here (in the parent process os.environ) guarantees every child
# process inherits LD_PRELOAD, unlike Node(additional_env=…) which is applied
# after the fork and may not take effect before DT_NEEDED resolution.
_RVIZ_PRELOAD = "/opt/ros/humble/lib/librviz_default_plugins.so"
os.environ["LD_PRELOAD"] = (
    _RVIZ_PRELOAD + (":" + os.environ["LD_PRELOAD"] if os.environ.get("LD_PRELOAD") else "")
)


def launch_setup(context, *_args, **_kwargs):
    pkg_share = Path(__file__).resolve().parents[1]
    hardware_type = LaunchConfiguration("hardware_type").perform(context)
    enable_servo = LaunchConfiguration("enable_servo").perform(context).lower() in {"true", "1", "yes"}
    enable_joystick = LaunchConfiguration("enable_joystick").perform(context).lower() in {"true", "1", "yes"}
    use_rviz = LaunchConfiguration("use_rviz")
    cleanup_existing = LaunchConfiguration("cleanup_existing")
    use_sim_time_arg = LaunchConfiguration("use_sim_time").perform(context).lower()
    if use_sim_time_arg in ("", "auto"):
        use_sim_time = hardware_type == "isaac"
    else:
        use_sim_time = use_sim_time_arg in {"true", "1", "yes"}
    use_sim_time_val = "true" if use_sim_time else "false"
    allow_fake_tool = hardware_type not in ("real", "twin")

    joint_commands_topic, joint_states_topic = joint_topics_for_hardware(hardware_type)
    xacro_hardware_type = normalize_xacro_hardware_type(hardware_type)
    filter_joint_states = use_filtered_joint_states(hardware_type)

    moveit_config = (
        MoveItConfigsBuilder("dual_arm_world", package_name="dual_arm_moveit_config")
        .robot_description(
            file_path="config/dual_arm_world.urdf.xacro",
            mappings={
                "hardware_type": xacro_hardware_type,
                "joint_commands_topic": joint_commands_topic,
                "joint_states_topic": joint_states_topic,
            },
        )
        .robot_description_semantic(file_path="config/dual_arm_world.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            str(pkg_share / "config" / "ros2_controllers.yaml"),
            {"use_sim_time": use_sim_time_val == "true"},
        ],
        output="screen",
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time_val == "true"}],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        remappings=[("joint_states", "filtered_joint_states")] if filter_joint_states else [],
        parameters=[
            moveit_config.to_dict(),
            {
                "default_planning_pipeline": "ompl",
                "use_sim_time": use_sim_time_val == "true",
                # publish_state_updates=False: joint-state changes (100 Hz from
                # joint_state_broadcaster) must NOT retrigger /monitored_planning_scene
                # republication.  Each republication causes RViz to re-render all
                # interactive markers → blinking / drag instability.  RViz already
                # shows the live robot state via its own /joint_states subscription
                # (RobotModel display), so disabling this loses nothing visually.
                # Only geometry changes (collision objects) will trigger republication.
                "publish_planning_scene_hz": 2.0,
                "publish_geometry_updates": True,
                "publish_state_updates": False,
                "publish_transforms_updates": False,
            },
        ],
    )

    # In Isaac mode only, remap /joint_states to /filtered_joint_states inside
    # RViz so unexpected names cannot reach MoveIt's RobotState. The adapter
    # has already normalized the USD joint names and coordinates. Twin mode
    # does not need this final filter because real feedback uses model names.
    rviz_remappings = [("joint_states", "filtered_joint_states")] if filter_joint_states else []
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(pkg_share / "rviz" / "moveit.rviz")],
        ros_arguments=["--log-level", "ERROR"],
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time_val == "true"}],
        remappings=rviz_remappings,
        additional_env={"LD_PRELOAD": _RVIZ_PRELOAD},
        condition=IfCondition(use_rviz),
    )

    joint_state_filter_node = Node(
        package="dual_arm_moveit_config",
        executable="joint_state_filter.py",
        name="joint_state_filter",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_val == "true"}],
    )

    # Digital-twin relay: mirrors /robot_joint_states → /isaac_joint_commands
    # with uf_slide_joint offset (URDF → Isaac coords) so Isaac Sim follows
    # the real robot in real time.
    isaac_state_relay_node = Node(
        package="dual_arm_moveit_config",
        executable="isaac_state_relay.py",
        name="isaac_state_relay",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_val == "true"}],
    )

    real_hardware_bridge = Node(
        package="dual_arm_moveit_config",
        executable="real_hardware.py",
        name="real_hardware_bridge",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    isaac_joint_adapter_node = Node(
        package="dual_arm_moveit_config",
        executable="isaac_joint_adapter.py",
        name="isaac_joint_adapter",
        parameters=[{"publish_clock": True}],
        output="screen",
    )

    tool_config_path = Path(get_package_share_directory("tool_controller")) / "config" / "tool_params.yaml"
    tool_commander = Node(
        package="tool_controller",
        executable="tool_commander",
        name="tool_commander",
        output="screen",
        parameters=[
            str(tool_config_path),
            {
                "use_sim_time": use_sim_time_val == "true",
                "allow_fake": allow_fake_tool,
            },
        ],
    )

    with open(pkg_share / "config" / "xarm_servo.yaml", "r", encoding="utf-8") as file:
        xarm_servo_params = yaml.safe_load(file)
    with open(pkg_share / "config" / "uf_servo.yaml", "r", encoding="utf-8") as file:
        uf_servo_params = yaml.safe_load(file)

    xarm_servo = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="xarm_servo_node",
        remappings=[("joint_states", "filtered_joint_states")] if filter_joint_states else [],
        parameters=[
            {"moveit_servo": xarm_servo_params},
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time_val == "true"},
        ],
        output="screen",
    )

    uf_servo = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="uf_servo_node",
        remappings=[("joint_states", "filtered_joint_states")] if filter_joint_states else [],
        parameters=[
            {"moveit_servo": uf_servo_params},
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time_val == "true"},
        ],
        output="screen",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{"use_sim_time": use_sim_time_val == "true"}],
        output="screen",
    )

    teleop_bridge = Node(
        package="dual_arm_moveit_config",
        executable="teleop_bridge.py",
        name="teleop_bridge",
        parameters=[{"use_sim_time": use_sim_time_val == "true"}],
        output="screen",
    )

    controller_manager_args = [
        "--controller-manager",
        "/controller_manager",
        "--controller-manager-timeout",
        "120",
        "--service-call-timeout",
        "60",
    ]
    controller_spawners = [
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", *controller_manager_args], output="screen"),
        Node(package="controller_manager", executable="spawner", arguments=["uf850_controller", *controller_manager_args], output="screen"),
        Node(package="controller_manager", executable="spawner", arguments=["xarm5_controller", *controller_manager_args], output="screen"),
        Node(package="controller_manager", executable="spawner", arguments=["uf850_servo_controller", "--inactive", *controller_manager_args], output="screen"),
        Node(package="controller_manager", executable="spawner", arguments=["xarm5_servo_controller", "--inactive", *controller_manager_args], output="screen"),
        Node(package="controller_manager", executable="spawner", arguments=["rg6_controller", *controller_manager_args], output="screen"),
        Node(package="controller_manager", executable="spawner", arguments=["slider_controller", *controller_manager_args], output="screen"),
    ]

    cleanup_patterns = [
        "dual_arm_moveit_config/launch/demo.launch.py",
        "dual_arm_moveit_config/launch/exotica.launch.py",
        "dual_arm_moveit_config/hardware/real_hardware.py",
        "dual_arm_moveit_config/hardware/joint_state_filter.py",
        "dual_arm_moveit_config/hardware/isaac_joint_adapter.py",
        "dual_arm_moveit_config/hardware/isaac_state_relay.py",
        "dual_arm_moveit_config/hardware/teleop_bridge.py",
        "dual_arm_moveit_config/dual_arm_moveit_config/exotica_ik_server_node.py",
        "moveit_servo/servo_node_main",
        "/opt/ros/humble/lib/joy/joy_node",
        "/opt/ros/humble/lib/controller_manager/ros2_control_node",
        "/moveit_ros_move_group/move_group",
        "/opt/ros/humble/lib/rviz2/rviz2",
        "/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher",
        "/opt/ros/humble/lib/tf2_ros/static_transform_publisher",
        "/opt/ros/humble/lib/controller_manager/spawner",
        "tool_commander",
    ]
    cleanup_script = "\n".join(
        [
            "import os",
            "import signal",
            "import subprocess",
            f"patterns = {cleanup_patterns!r}",
            "me = os.getpid()",
            "out = subprocess.check_output(['ps', '-eo', 'pid=,args='], text=True)",
            "targets = []",
            "for raw in out.splitlines():",
            "    line = raw.strip()",
            "    if not line:",
            "        continue",
            "    parts = line.split(None, 1)",
            "    pid = int(parts[0])",
            "    args = parts[1] if len(parts) > 1 else ''",
            "    if pid == me:",
            "        continue",
            "    if any(pattern in args for pattern in patterns):",
            "        targets.append(pid)",
            "for pid in targets:",
            "    try:",
            "        os.kill(pid, signal.SIGTERM)",
            "    except ProcessLookupError:",
            "        pass",
        ]
    )
    cleanup_existing_processes = ExecuteProcess(
        cmd=["python3", "-c", cleanup_script],
        output="screen",
        condition=IfCondition(cleanup_existing),
    )

    actions = [
        LogInfo(msg="Cleaning up stale dual-arm ROS processes before launch", condition=IfCondition(cleanup_existing)),
        cleanup_existing_processes,
        LogInfo(msg=f"Starting dual-arm stack in {hardware_type} hardware mode"),
    ]
    if hardware_type == "isaac":
        actions.append(isaac_joint_adapter_node)
    actions.append(TimerAction(period=1.0, actions=[rsp_node, ros2_control_node]))

    if hardware_type in ("real", "twin"):
        actions.append(TimerAction(period=1.0, actions=[real_hardware_bridge]))

    if hardware_type == "isaac":
        # The adapter has already normalized USD names and coordinates.
        # This final filter protects MoveIt from any unexpected joint names.
        # RViz is remapped to subscribe to /filtered_joint_states.
        actions.append(TimerAction(period=2.0, actions=[joint_state_filter_node]))

    if hardware_type == "twin":
        # Digital twin: relay real robot states to Isaac Sim so it mirrors
        # the physical robot in real time.  No joint_state_filter here —
        # the real robot publishes URDF-coordinate joint states with no
        # unknown joints, so RViz subscribes to /joint_states directly.
        # The real hardware state path does not require Isaac normalization.
        actions.append(TimerAction(period=2.0, actions=[isaac_state_relay_node]))

    # Staggered spawning to avoid swamping the controller manager service executor
    spawner_delay = 20.0
    for spawner in controller_spawners:
        actions.append(
            TimerAction(
                period=spawner_delay,
                actions=[spawner],
            )
        )
        spawner_delay += 1.0
    actions.append(
        TimerAction(
            period=5.0,
            actions=[LogInfo(msg="Starting MoveIt"), move_group_node],
        )
    )

    if enable_servo:
        servo_actions = [xarm_servo, uf_servo]
        servo_log = "Starting MoveIt Servo"
        if enable_joystick:
            servo_actions.extend([joy_node, teleop_bridge])
            servo_log = "Starting MoveIt Servo and teleop"
        actions.append(
            TimerAction(
                period=7.0,
                actions=[LogInfo(msg=servo_log), *servo_actions],
            )
        )
    actions.append(
        TimerAction(
            period=30.0,
            actions=[LogInfo(msg="Starting RViz"), rviz_node],
        )
    )

    actions.append(
        TimerAction(
            period=9.0 if enable_servo else 7.0,
            actions=[LogInfo(msg="Starting tool controller"), tool_commander],
        )
    )

    return actions


def generate_launch_description():
    ld = LaunchDescription()
    moveit_pkg_share = Path(get_package_share_directory("dual_arm_moveit_config"))
    ld.add_action(DeclareBooleanLaunchArg("db", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("enable_servo", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("enable_joystick", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("cleanup_existing", default_value=True))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="auto"))
    ld.add_action(DeclareLaunchArgument("hardware_type", default_value="fake"))
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
