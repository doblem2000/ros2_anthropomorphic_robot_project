import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, LogInfo, OpaqueFunction, GroupAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

def spawn_controller_list(context):
    return [
        Node(package="controller_manager",
            executable="spawner",
            arguments=[controller, "--controller-manager", "/controller_manager"],
        )
        for controller in context.launch_configurations["controller_list"].split()
    ]


def generate_launch_description():
    # These are the controller whe use when planning with MoveIt
    controller_list = "fanuc_m20ia_35m_joint_trajectory_controller"

    # Value argument from launch configuration
    gui = LaunchConfiguration("gui")
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
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
            "command_interface:=velocity",
            " ",
            "ros2_controllers_path:=", 
            ros2_controllers_file
        ],
        on_stderr='ignore' #TODO: Understand why sometimes xacro prints to stderr
    )
    robot_description = {"robot_description": robot_description_content}

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
    
    move_group_launch_file = PathJoinSubstitution(
        [
            FindPackageShare('acg_resources_fanuc_m20ia_35m_moveit_config'),
            'launch',
            'move_group.launch.py'
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("acg_resources_fanuc_m20ia_35m_controllers"), 
            "config", 
            "task_space_control_demo.rviz"
        ]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_file]
    )

    # Include Gazebo launch file to launch its environment
    launches = []

    launches.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments=[('gz_args', ['-r -v 4 empty.sdf'])],
        )
    )
    
    move_group_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch_file),
        launch_arguments=[('moveit_manage_controllers', 'false')],
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    task_space_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fanuc_m20ia_35m_task_space_controller", "--controller-manager", "/controller_manager"],
    )
    
    reference_generator_node = Node(
        package="task_space_trajectory_controller",
        executable="reference_generator",
        output="screen",
        parameters=[{"base_link": "world"},
                    {"end_effector_link": "fanuc_m20ia_35m_drilling_tool"},
                    {"command_topic": "/fanuc_m20ia_35m_task_space_controller/command"},],
    )
    
    # Define GroupAction actions (needed to enforce use_sim_time)
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
    
    # Delay controllers start after spawning the model in Gazebo
    delay_controllers_after_ignition_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ignition_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, task_space_controller_spawner]
        )
    )
    
    # Delay RViz start after `joint_state_broadcaster`
    delay_rviz_after_controllers_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    
    # Delay reference generator start after RViz
    delay_reference_generator_rviz = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=rviz_node,
            on_start=[reference_generator_node],
        )
    )

    nodes = [
        ignition_spawn_entity,
        delay_controllers_after_ignition_spawn,
        delay_rviz_after_controllers_spawner,
        delay_reference_generator_rviz
    ]

    return LaunchDescription(declared_arguments + launches + group_actions + nodes)