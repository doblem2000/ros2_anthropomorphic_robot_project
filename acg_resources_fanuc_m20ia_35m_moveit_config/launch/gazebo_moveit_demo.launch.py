import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, LogInfo, OpaqueFunction, GroupAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "command_interface",
            default_value="position",
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

    rviz_launch_file = PathJoinSubstitution(
        [
            FindPackageShare('acg_resources_fanuc_m20ia_35m_moveit_config'),
            'launch',
            'moveit_rviz.launch.py'
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
    
    move_group_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(move_group_launch_file),
        launch_arguments=[('moveit_manage_controllers', 'false')],
    ) 


    rviz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_file),
        condition=IfCondition(gui)
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
    
    # # This is spawned only temporarily to recover from the fall
    # # under the effect of gravity. Therefore we define the spawner...
    # forward_position_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["fanuc_m20ia_35m_forward_position_controller", "--controller-manager", "/controller_manager"],
    # )

    # # ... and the unspawner
    # forward_position_controller_unspawner = Node(
    #     package="controller_manager",
    #     executable="unspawner",
    #     arguments=["fanuc_m20ia_35m_forward_position_controller", "--controller-manager", "/controller_manager"],
    # )
    

    trajectory_controller_spawner = Node(package="controller_manager",
            executable="spawner",
            arguments=["fanuc_m20ia_35m_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        )

    # # This is the temporary reference generator communicating with the
    # # forward position controller to recover from the fall
    # topic_echo_process = ExecuteProcess(
    #     cmd=['ros2', 'topic', 'pub', '-t 3',
    #          '/fanuc_m20ia_35m_forward_position_controller/commands',
    #          'std_msgs/msg/Float64MultiArray',
    #          '{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'],
    #     name='bring_fanuc_m20ia_35m_to_home',
    #     output='both'
    # )
    
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
            on_exit=[joint_state_broadcaster_spawner, trajectory_controller_spawner]
        )
    )
    
    # # Send command to recover from the fall
    # # after the forward position controller has been activated
    # delay_topic_echo_after_forward_position_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=forward_position_controller_spawner,
    #         on_exit=topic_echo_process
    #     )
    # )
    
    # # Unspawn forward position controller after placing the robot at the desired configuration
    # delay_unspawner_after_topic_echo = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=topic_echo_process,
    #         on_exit=[forward_position_controller_unspawner]
    #     )
    # )

    # # After unloading the forward position controller, the
    # # trajectory controller can be activated
    # delay_trajectory_controller_spawner_after_unspawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=forward_position_controller_unspawner,
    #         on_exit=trajectory_controller_spawner
    #     )
    # )
    
    # Delay RViz start after `joint_state_broadcaster`
    delay_rviz_after_controllers_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_launch_description],
        )
    )

    nodes = [
        ignition_spawn_entity,
        delay_controllers_after_ignition_spawn,
        # delay_topic_echo_after_forward_position_controller_spawner,
        # delay_unspawner_after_topic_echo,
        # delay_trajectory_controller_spawner_after_unspawner,
        delay_rviz_after_controllers_spawner,
    ]

    return LaunchDescription(declared_arguments + launches + group_actions + nodes)