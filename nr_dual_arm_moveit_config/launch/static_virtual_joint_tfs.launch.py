from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("nr_dual_arm", package_name="nr_dual_arm_moveit_config")
        .robot_description_semantic(file_path="config/nr_dual_arm.srdf")
        .to_moveit_configs()
    )
    return generate_static_virtual_joint_tfs_launch(moveit_config)
