from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from nr_dual_arm_moveit_config.runtime_config import (
    joint_topics_for_hardware,
    normalize_xacro_hardware_type,
    use_filtered_joint_states,
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
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    entities = generate_move_group_launch(moveit_config).entities
    for entity in entities:
        if hasattr(entity, "parameters"):
            entity.parameters = list(entity.parameters) + [
                {"default_planning_pipeline": "ompl"},
                {"use_sim_time": use_sim_time},
            ]
        if filter_joint_states and hasattr(entity, "remappings"):
            entity.remappings = list(entity.remappings) + [("joint_states", "filtered_joint_states")]
    return entities


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("hardware_type", default_value="fake"))
    ld.add_action(DeclareLaunchArgument("use_sim_time", default_value="auto"))
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
