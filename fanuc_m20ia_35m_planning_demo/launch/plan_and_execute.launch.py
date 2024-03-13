from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    launch_description = []
    
    moveit_config = MoveItConfigsBuilder("fanuc_m20ia_35m", package_name="acg_resources_fanuc_m20ia_35m_moveit_config").to_moveit_configs()
    
    # Declare the launch argument
    preset_config_param = DeclareLaunchArgument("config",
                                             description="Name of the configuration file to use without the .yaml extension")
    launch_description.append(preset_config_param)
    path_bagfile_param = DeclareLaunchArgument("bagfile_path",
                                             description="Name of the bagfile containing the path to be planned and executed")
    launch_description.append(path_bagfile_param)
    config_file_name_param = DeclareLaunchArgument("config_file_name", default_value=[LaunchConfiguration("config"),'.yaml'])
    launch_description.append(config_file_name_param)

    # Get the launch argument
    config_file_name = LaunchConfiguration('config_file_name')
    bagfile_path = LaunchConfiguration('bagfile_path')
    config_file_path = PathJoinSubstitution(
        [FindPackageShare(package='fanuc_m20ia_35m_planning_demo'),
        'config',
        config_file_name])

    # Create the node
    planning_node_action = Node(
    package='fanuc_m20ia_35m_planning_demo',
    executable='fanuc_m20ia_35m_planning_demo',
    output='screen',
    arguments=[bagfile_path],
    parameters=[config_file_path,
                moveit_config.robot_description,
                moveit_config.robot_description_kinematics,
                moveit_config.robot_description_semantic])
    launch_description.append(planning_node_action)
    return LaunchDescription(launch_description)