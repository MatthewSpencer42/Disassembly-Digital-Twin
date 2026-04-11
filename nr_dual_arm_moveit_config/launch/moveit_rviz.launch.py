from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from nr_dual_arm_moveit_config.runtime_config import (
    joint_topics_for_hardware,
    normalize_xacro_hardware_type,
    use_filtered_joint_states,
)


_RVIZ_PRELOAD = "/opt/ros/humble/lib/librviz_default_plugins.so"
os.environ["LD_PRELOAD"] = (
    _RVIZ_PRELOAD + (":" + os.environ["LD_PRELOAD"] if os.environ.get("LD_PRELOAD") else "")
)


def _launch_setup(context, *_args, **_kwargs):
    hardware_type = LaunchConfiguration("hardware_type").perform(context)
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    if use_sim_time_str in ("", "auto"):
        use_sim_time = (hardware_type == "isaac")
    else:
        use_sim_time = use_sim_time_str.lower() in {"true", "1", "yes"}
    joint_commands_topic, joint_states_topic = joint_topics_for_hardware(hardware_type)
    xacro_joint_commands_topic = (
        "/isaac_joint_commands_urdf" if hardware_type == "isaac" else joint_commands_topic
    )
    xacro_hardware_type = normalize_xacro_hardware_type(hardware_type)
    filter_joint_states = use_filtered_joint_states(hardware_type)
    moveit_config = (
        MoveItConfigsBuilder("nr_dual_arm", package_name="nr_dual_arm_moveit_config")
        .robot_description(
            file_path="config/nr_dual_arm.urdf.xacro",
            mappings={
                "hardware_type": xacro_hardware_type,
                "joint_commands_topic": xacro_joint_commands_topic,
                "joint_states_topic": joint_states_topic,
            },
        )
        .robot_description_semantic(file_path="config/nr_dual_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            remappings=[("joint_states", "filtered_joint_states")] if filter_joint_states else [],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {"use_sim_time": use_sim_time},
            ],
        )
    ]


def generate_launch_description():
    package_path = Path(__file__).resolve().parents[1]
    ld = LaunchDescription()
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareLaunchArgument("hardware_type", default_value="fake"))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="auto"))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(package_path / "config" / "moveit.rviz"),
        )
    )
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
