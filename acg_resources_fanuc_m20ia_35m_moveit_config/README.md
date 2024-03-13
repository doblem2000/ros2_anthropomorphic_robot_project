# acg_resources_fanuc_m20ia_35m_moveit_config

This package contains the Fanuc M20ia/35M robot configuration preferred by our group. It has been created through the MoveIt! Setup Assistant and then manually modified. Since the Setup Assistant will detect manual modifications, additional functions shall be configured by manually editing the available launch and config files.

### Setup Assistant configuration
Some of the decision made during the creation of this package are listed here below:
- The chosen solver for each planning group is KDL;
- The chosen planner for each planning group is OMPL with the BiTRRT algorithm;
- joint_limits.yaml are set to the real robot's limits (due to the presence of the end effector, the limits are different from the ones reported in the robot's manual);
- We added a single JointTrajectoryController to control the robot with the `position` command interface;
- We added a JointStateBroadcaster to publish the robot's joint states;
- We defined 4 planning groups:
    - `fanuc_m20ia_35m_slider_to_drilling_tool`: The planning group used to plan the motion of the slider to the drilling tool;
    - `fanuc_m20ia_35m_slider_to_inspection_tool`: The planning group used to plan the motion of the slider to the inspection tool;
    - `fanuc_m20ia_35m_base_to_drilling_tool`: The planning group used to plan the motion of the base to the drilling tool, without considering the slide;
    - `fanuc_m20ia_35m_base_to_inspection_tool`: The planning group used to plan the motion of the base to the inspection tool, without considering the slide;
- For each planning group, we defined the proper end effector link as either the drilling tool or the inspection tool;
- For each planning group, we defined two home positions:
    - `home`: The home position of the robot, with the arm facing to the front of the slide;
    - `home_inverse`: The inverse of the previous position, with the arm facing to the back of the slide;

### Manual modifications
Some of the modifications applied to this package are listed here below:
- The xacro file used to generate the URDF model of the robot has been modified to allow to dynamically change the following:
    - `use_mock_hardware`: Whether to use the `mock_components/GenericSystem` hardware interface or gazebo's `ign_ros2_control/IgnitionSystem` hardware interface;
    - `command_interface`: Which command interface to use (`position`, `velocity`, `effort`);
    - `ros2_controllers_path`: The path of the configuration file `ros2_controllers.yaml` to be used by the `ign_ros2_control-system` plugin;
    - `initial_positions_file`: The initial positions file;
    - `initial_velocities_file`: The initial velocities file;
    - `initial_efforts_file`: The initial efforts file;
- The initial configuration of the robot is set to a custom one (see `initial_positions.yaml`, `initial_velocities.yaml`, `initial_efforts.yaml`);
- Thee planning pipelines are provided: `ompl`, `chomp` and `pilz_industrial_planner`.

# Installing the dependencies
In order to install the needed dependencies run the command
```bash
rosdep install --from-paths src --ignore-src -r -y
```
inside the project's root folder

# Launch files
## Usage with Motion Planning plugin and Mock Hardware
In order to execute a demo with RViz's MotionPlanning plugin, run
```bash
ros2 launch acg_resources_fanuc_m20ia_35m_moveit_config demo.launch.py
```
the demo launches the mock hardware and the MoveIt! RViz plugin, which allows the user to plan and execute trajectories for the robot. The execution is performed by a `JointTrajectoryController`, which is managed by the `moveit_simple_controller_manager`, which, despite its name, provides action clients connecting to the controllers loaded by the (real) controller manager.

## Usage with external planning application and Mock Hardware
The only difference with the previous modality is the RViz's configuration, which loads `MarkerArray` and `Trajectory` plugins for external applications to effectively visualize the results of planning and execution:
```bash
ros2 launch acg_resources_fanuc_m20ia_35m_moveit_config demo.launch.py rviz_config:=./install/acg_resources_fanuc_m20ia_35m_moveit_config/share/acg_resources_fanuc_m20ia_35m_moveit_config/config/ros2_control.rviz
```

## Simulation with Gazebo and an external reference generator
This demo is the same as the demo above, but ign_ros2_control::IgnitionROS2ControlPlugin plugin is used instead of launching a dedicated ros2_control_node. This means that the control node (and, therefore, the controller manager) is started by Gazebo at the time the plugin is loaded. Also, in terms of hardware, the ign_ros2_control/IgnitionSystem is used instead of mock hardware.
In this case, with do not use MoveIt! to plan and execute trajectories, but we can still send references to the joint trajectory controller through the ros2 topic pub command. Notice that this launch file does not load `move_group`.

To launch the demo, run
```bash
ros2 launch acg_resources_fanuc_m20ia_35m_moveit_config gazebo_ros2_control_demo.launch.py
```

Position references can be sent to the Gazebo robot by an external reference generator, such as ros2 topic pub, e.g.

```bash
ros2 topic pub -1 /fanuc_m20ia_35m_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{\
  header: {\
    stamp: {sec: 0, nanosec: 0},\
    frame_id: world\
  },\
  joint_names: [fanuc_m20ia_35m_joint_1, fanuc_m20ia_35m_joint_2, fanuc_m20ia_35m_joint_3, fanuc_m20ia_35m_joint_4, fanuc_m20ia_35m_joint_5, fanuc_m20ia_35m_joint_6],\
  points: [\
    {\
      positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],\
      velocities: [],\
      accelerations: [],\
      effort:[],\
      time_from_start: {sec: 0, nanosec: 0}\
    },\
    {\
      positions: [-3.14, 1.57, 1.57, 0.0, 1.047, 0.0],\
      velocities: [],\
      accelerations: [],\
      effort:[],\
      time_from_start: {sec: 2, nanosec: 0}\
    }\
  ]\
}"
```


## Simulation with Gazebo and MoveIt! planning system
This package also offers a complete planning and simulation setup with the following demo:
```bash
ros2 launch acg_resources_fanuc_m20ia_35m_moveit_config gazebo_moveit_demo.launch.py
```
Here, the user can plan trajectories through the MoveIt! planning pipeline by using the MotionPlanning plugin in RViz. Execution is performed on the Gazebo robot through the moveit_simple_controller_manager, which uses the loaded joint trajectory controller.
