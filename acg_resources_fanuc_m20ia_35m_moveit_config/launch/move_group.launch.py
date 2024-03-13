from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch_param_builder import load_yaml
from pathlib import Path
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch(context, moveit_config):
    if "trajectory_execution_config_path" in context.launch_configurations:
        path = Path(context.launch_configurations["trajectory_execution_config_path"])
        print("Trajectory execution config path found: ", str(path))
        moveit_config.trajectory_execution = load_yaml(path)
    return generate_move_group_launch(moveit_config).entities

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fanuc_m20ia_35m", package_name="acg_resources_fanuc_m20ia_35m_moveit_config").to_moveit_configs()
    return LaunchDescription([OpaqueFunction(function=generate_launch, args=[moveit_config])])
