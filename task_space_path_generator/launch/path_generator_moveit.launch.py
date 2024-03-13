import yaml
from os import path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Shutdown, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, TextSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def get_launch_files(context, ouroboros_config_file_path,rviz_config_file_path):
    ouroboros_config_file_path = ouroboros_config_file_path.perform(context)
    rviz_config_file_path = rviz_config_file_path.perform(context)
    print(ouroboros_config_file_path)
    with open(ouroboros_config_file_path, 'r') as stream:
        try:
            config_file = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    moveit_config = config_file['launchfiles']['ros__parameters']['moveit_config_package']
    return [IncludeLaunchDescription([
        FindPackageShare(package=moveit_config['package_name']),
        "/launch/",
        moveit_config['launchfile_name']],
                                     launch_arguments={'rviz_config': rviz_config_file_path}.items())]

def generate_launch_description():
    params_declarations = []
    
    robot_name_param = DeclareLaunchArgument("robot_name",
                                             default_value="fanuc_m20ia_35m",
                                             description="Name of the robot")
    config_file_name_param = DeclareLaunchArgument("config_file_name", default_value=[LaunchConfiguration("robot_name"),'.yaml'])
    rviz_config_file_name_param = DeclareLaunchArgument("rvizconfig", default_value=[LaunchConfiguration("robot_name"),'_moveit.rviz'])
    params_declarations.append(robot_name_param)
    params_declarations.append(config_file_name_param)
    params_declarations.append(rviz_config_file_name_param)
    
    ouroboros_config_file_path = PathJoinSubstitution(
        [FindPackageShare(package='task_space_path_generator'),
        'config', LaunchConfiguration("config_file_name")])
    
    rviz_config_file_path = PathJoinSubstitution(
        [FindPackageShare(package='task_space_path_generator'),
        'config', LaunchConfiguration("rvizconfig")])
    
    run_launch = OpaqueFunction(function=get_launch_files, args=[ouroboros_config_file_path,rviz_config_file_path])
    
    ouroboros_gui_node = Node(package="task_space_path_generator",
                              executable="ouroboros_gui",
                              parameters=[ouroboros_config_file_path])
    
    return LaunchDescription(params_declarations + [run_launch, ouroboros_gui_node])