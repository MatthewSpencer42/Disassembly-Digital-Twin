from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch
from nr_dual_arm_moveit_config.runtime_config import joint_topics_for_hardware, normalize_xacro_hardware_type


def _launch_setup(context, *_args, **_kwargs):
    hardware_type = LaunchConfiguration("hardware_type").perform(context)
    joint_commands_topic, joint_states_topic = joint_topics_for_hardware(hardware_type)
    xacro_hardware_type = normalize_xacro_hardware_type(hardware_type)
    moveit_config = (
        MoveItConfigsBuilder("nr_dual_arm", package_name="nr_dual_arm_moveit_config")
        .robot_description(
            file_path="config/nr_dual_arm.urdf.xacro",
            mappings={
                "hardware_type": xacro_hardware_type,
                "joint_commands_topic": joint_commands_topic,
                "joint_states_topic": joint_states_topic,
            },
        )
        .to_moveit_configs()
    )
    return generate_rsp_launch(moveit_config).entities


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("hardware_type", default_value="fake"))
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
