import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, LogInfo, OpaqueFunction, GroupAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit,OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution

def spawn_controller_list(context):
    return [
        Node(package="controller_manager",
            executable="spawner",
            arguments=[controller, "--controller-manager", "/controller_manager", "--controller-manager-timeout", "60"],
        )
        for controller in context.launch_configurations["controller_list"].split()
    ]


def generate_launch_description():

    # Value argument from launch configuration
    gui = LaunchConfiguration("gui")
    controller_list = LaunchConfiguration("controller_list")

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_list",
            default_value="fanuc_m20ia_35m_PID_controller",
            description="Load the effort joint controllers"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "command_interface",
            default_value="effort",
            description="The command interface to be used to control the Ignition System."
        )
    )
    
    # Get URDF via xacro
    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("acg_resources_fanuc_m20ia_35m_moveit_config"),
            "config",
            "fanuc_m20ia_35m.urdf.xacro",
        ]
    )
    
    ros2_controllers_file = PathJoinSubstitution(
        [
            FindPackageShare("acg_resources_fanuc_m20ia_35m_controllers"),
            "config",
            "ros2_controllers.yaml",
        ]
    )

    # Load robot description without mock hardware
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "use_mock_hardware:=false",
            " ",
            "command_interface:=",
            LaunchConfiguration("command_interface"),
            " ",
            "ros2_controllers_path:=", 
            ros2_controllers_file
        ],
        on_stderr='ignore' #TODO: Understand why sometimes xacro prints to stderr
    )
    robot_description = {"robot_description": robot_description_content}

    # Load RViz configuration without planning plugins
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("acg_resources_fanuc_m20ia_35m_moveit_config"), 
            "config", 
            "ros2_control.rviz"
        ]
    )

    # Prepare Gazebo resources and launch path:
    # this is needed by Gazebo to find the meshes referenced by the URDF
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = get_package_share_directory("acg_resources_fanuc_m20ia_35m_description") + "/.."

    gazebo_launch_file = PathJoinSubstitution(
        [
            FindPackageShare('ros_ign_gazebo'),
            'launch', 
            'ign_gazebo.launch.py'
        ]
    )

    # Include Gazebo launch file to launch its environment
    launches = []

    launches.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments=[('gz_args', ['-r -v 4 empty.sdf'])],
        )
    )

    # Define all nodes that have to be launched
    
    # Spawn the model in Gazebo. This also loads the plugin and starts the control node.
    # At the time of writing this script, the 'create' command does not allow spawning
    # the robot at a specific joint configuration
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description_content,
                   '-name', 'fanuc_m20ia_35m',
                   '-allow_renaming', 'true',
                   '-z','0.0001'], # This is very important because the slider model slightly compenetrates the ground
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    motion_controller_spawner = OpaqueFunction(function=spawn_controller_list)

    # Delay `joint_state_broadcaster` start after spawning the model in Gazebo
    delay_controllers_after_ignition_spawn = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ignition_spawn_entity,
            on_start=[motion_controller_spawner]
        )
    )

    delay_joint_state_broadcaster_spawner_after_ignition_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ignition_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    # Delay RViz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    
    move_group_launch_file = PathJoinSubstitution(
        [
            FindPackageShare('acg_resources_fanuc_m20ia_35m_moveit_config'),
            'launch',
            'move_group.launch.py'
        ]
    )
    trajectory_execution_config_file = PathJoinSubstitution(
        [
            FindPackageShare('acg_resources_fanuc_m20ia_35m_controllers'),
            'config',
            'moveit_controllers.yaml'
        ]
    )
    move_group_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch_file),
        launch_arguments=[('moveit_manage_controllers', 'false'), ('trajectory_execution_config_path', trajectory_execution_config_file)],
    ) 
    
    group_actions = []
    
    group_actions.append(
        GroupAction(
            actions=[
                SetParameter(name='use_sim_time', value=True),
                move_group_launch_description,
                robot_state_pub_node
            ]
        )
    )

    nodes = [
        ignition_spawn_entity,
        delay_controllers_after_ignition_spawn,
        delay_joint_state_broadcaster_spawner_after_ignition_spawn,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + launches + group_actions + nodes)