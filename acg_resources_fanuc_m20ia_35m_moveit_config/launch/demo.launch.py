from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fanuc_m20ia_35m", package_name="acg_resources_fanuc_m20ia_35m_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
