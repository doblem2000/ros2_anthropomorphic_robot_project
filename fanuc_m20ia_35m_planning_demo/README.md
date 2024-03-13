# ROS2 Planning Package for Fanuc M20ia_35m

## Overview

This ROS2 package streamlines path planning for the Fanuc M20ia_35m industrial robot using MoveIt!2. Its primary goal is to facilitate path planning simulations by taking a desired path as input, read from a BAG file. The planning process consists of three main steps:

1.  **Read the desired path from the bag file**
    * First of all, it is necessary to read the input bag file through the function ``read_cartesian_path_poses``.

2.  **Dynamic Home Trajectory Planning:**
    
    *   This step involves planning the trajectory that brings the robot to a "dynamic" home position.
3.  **Target Pose Planning:**
    
    *   The third step focuses on planning to a target pose, where the target pose is the first point of the trajectory obtained from the provided bag file. Planned trajectories are validated against the desired position, and planning fails if the distance exceeds a specific threshold. The validation is implemented through ``plan_validation`` method of the ``planning_node`` class.
4.  **Cartesian path Planning:**
    
    *   Finally, the last step deals with planning the trajectory that enables the robot to follow the desired path, as specified in the bag file. This step is performed by computing a cartesian path and to achieve this, the waypoints are populated with the points of the trajectory read from the bag file using the ``populate_waypoints``.

The home position is dynamic as it depends on the user-defined path's spatial position relative to the base frame: essentially, the robot adopts one of two home poses, referred to as ``home`` and ``home inverse`` based on which of the two allows it to approach the trajectory more "naturally". The package has been extended to generate task space references, not just joint space references. The user, through the choice of a parameter that will be detailed later, can decide whether to generate references in joint space or operational space. The generation of references in the task space is performed using a dedicated method named ``get_task_space_trajectory`` of the ``planning_node`` class that  orchestrates the generation of a Cartesian trajectory in the task space from a given joint trajectory. It leverages two essential private method of the afore mentioned class: the ``direct_kinematic`` method for deriving end-effector poses and the ``first_order_kinematic`` method for determining task space velocities. The resulting Cartesian trajectory is encapsulated in a ``moveit_msgs::msg::CartesianTrajectory`` message. An additional feature of the package is the recording of executed trajectories into a bag file, facilitating subsequent analysis. Trajectory execution is not achieved by directly invoking the execute method of the MoveGroupInterface class. Instead, actions are sent to the controller's action server using an action client, and the feedback messages returned during execution are serialized. These feedback messages provide essential information regarding the error between the planned trajectory and the one effectively executed. This methodology enables key insights into the trajectory execution process for further analysis and optimization. It is important to note that the action client is an attribute of the PlanningNode class. In particular, two types of action clients are implemented, each with their respective callbacks to handle the distinction between sending references in joint space and references in task space.

The package is designed with a modular structure to enable customization and extension of functionalities.

### Key Features

- **MoveIt!2 Path Planning**: Utilizes the MoveIt!2 framework for path planning, ensuring precise and safe robot movements.
- **Registered Data Support**: The package is engineered to work with registered data (bag file), allowing users to conduct experiments based on pre-registered paths.
- **Generation of both task space and joint space references for the controllers**:The package offers the flexibility to generate references in both task space and joint space, providing adaptability for different control strategies and scenarios.
-**Recording of trajectories carried out for any subsequent analysis**:Enables the recording of robot trajectories during executions, allowing users to perform in-depth analyses and optimizations based on real-world data.
- **Modular Structure**: The package is organized into distinct modules, making it easy to add new modules or extend existing functionalities.

## Package Structure

The package is structured with the following modules:

- **fanuc_m20ia_35m_planning_node**: The main node for MoveIt!2 path planning.
- **planning_node**: It handles communication with action servers for trajectory execution. Among its main functionalities, it can generate Cartesian trajectories, manage action sending, and record executed trajectories in a bag file for further analysis. 

## Installing the dependencies
 
In order to install the needed dependencies run the command

```bash
rosdep install --from-paths src --ignore-src -r -y
```
    
inside the project's root folder


### Node Parameters

### Base Frame Configuration

*   **Name**: base frame
*   **Description**: Specifies the name of the base frame.
*   **Default Value**: `"world"`

### Controller Name 

*   **Name**: controller name
*   **Description**: Specifies the name of the controller.

### Controller Type 

*   **Name**: controller type
*   **Description**: Specifies the type of controller, which can be either `TaskSpaceTrajectoryController` or `JointTrajectoryController`.

#### Feedback Sampling Period

*   **Name**: `feedback_sampling_period`
*   **Description**: Sampling period for feedback recording.
*   **Default Value**: `1`

#### Goal Joint Tolerance

*   **Name**: `goal_joint_tolerance`
*   **Description**: Tolerance for goal joint.
*   **Default Value**: `0.000100`

#### Goal Position Tolerance

*   **Name**: `goal_position_tolerance`
*   **Description**: Tolerance for goal position.
*   **Default Value**: `0.000100`

#### Goal Orientation Tolerance

*   **Name**: `goal_orientation_tolerance`
*   **Description**: Tolerance for goal orientation.
*   **Default Value**: `0.000100`

#### Input Bag File Topic

*   **Name**: `input_bag_file_topic`
*   **Description**: Specifies the topic name for the input bag file.
*   **Default Value**: `"cartesian_path"`

#### Max Velocity Scaling Factor

*   **Name**: `max_velocity_scaling_factor`
*   **Description**: Scaling factor for maximum velocity.
*   **Default Value**: `0.5`

#### Named Targets

*   **Name**: `named_targets`
*   **Description**: A structured parameter that contains: home_pose and home_pose_inverse.
*   **Sub-parameters**:
    *   **home_pose**: Specifies the home pose.
    *   **home_pose_inverse**: Specifies the inverse home pose.
*   **Default Value**: depends on the specific named targets defined into SRDF.

#### Planning Group

*   **Name**: `planning_group`
*   **Description**: Defines the planning group for path planning.
*   **Default Value**: `"fanuc_m20ia_35m_slider_to_drilling_tool"`

#### Recorded Trajectory Cartesian Topic

*   **Name**: `recorded_trajectory_cartesian_topic`
*   **Description**: Topic for recording Cartesian trajectory feedback.
*   **Default Value**: `"/fanuc_m20ia_cartesian_feedback"`

#### Recorded Trajectory File Name

*   **Name**: `recorded_trajectory_file_name`
*   **Description**: Specifies the file name for recorded trajectories.
*   **Default Value**: `"executed_trajectory"`

#### Recorded Trajectory Home Pose Topic

*   **Name**: `recorded_trajectory_home_pose_topic`
*   **Description**: Specifies the topic for recording home pose trajectory feedback.
*   **Default Value**: `"/home_trajectory_feedback"`

#### Recorded Trajectory Point-to-Point Topic

*   **Name**: `recorded_trajectory_point2point_topic`
*   **Description**: Topic for recording point-to-point trajectory feedback.
*   **Default Value**: `"/point2point_feedback"`

***

### Important Note

*   Parameters are essential for proper node functioning. Ensure correct values are set before deploying the node. Incorrect configurations may lead to unexpected behavior.


## Launch files
This package serves as a tool for executing planning operations. The functionality of the node is adjustable through parameters, providing flexibility to cater to various planning needs. While the package comes with pre-configured settings, it also offers the possibility for users to create their own configurations. This can be achieved by generating files that follow the structure of those located in the config directory.

The configurations provided within this package facilitate planning operations through several methods. These include the utilization of joint trajectory controller `joint_trajectory`, independent joint control `pid`, and task space control `task_space`. Additionally, users have the option to create new configurations by following the examples of the proposed ones.
The launch file in question includes a `config:=` parameter, which allows for switching between different configurations. Another parameter, `bagfile_path:=`, is used to specify the path to the bag file containing the trajectory to be planned.

In order to successfully launch the demonstrations provided with this package, it is necessary to operate two separate shell instances. The first shell is responsible for loading the entire ROS2 network. The second shell is dedicated to launching the planning node.

The following section will provide a detailed guide on how to use the provided launch files.

### Planning with joint trajectory controller on mock hardware
First we need to launch the mock hardware together with the MoveIt!2 `move_group` node. This can be achieved by executing the following command:
```bash
ros2 launch acg_resources_fanuc_m20ia_35m_moveit_config demo.launch.py rviz_config:=./install/acg_resources_fanuc_m20ia_35m_moveit_config/share/acg_resources_fanuc_m20ia_35m_moveit_config/config/ros2_control.rviz
```
This command launches all the necessary nodes for the MoveIt!2 framework to work. RViz is also launched with the configuration file specified by the `rviz_config` parameter. This configuration file is located in the `config` directory of the `acg_resources_fanuc_m20ia_35m_moveit_config` package and is needed to make the planning module work properly.

After that, we launch the planning node with the following command:
```bash
ros2 launch fanuc_m20ia_35m_planning_demo plan_and_execute.launch.py config:=joint_trajectory bagfile_path:=<path_to_bag_file>
```
where `<path_to_bag_file>` is the path to the bag file containing the trajectory to be planned.

### Planning with independent joint control on simulated hardware 
First we need to launch the simulated hardware together with the independent joint controller. This can be achieved by executing the following command:
```bash 
ros2 launch acg_resources_fanuc_m20ia_35m_controllers gazebo_ros2_control_demo.launch.py controller_list:='fanuc_m20ia_35m_PID_controller' command_interface:='effort'
```
Then we can launch the planning node with the following command:
```bash
ros2 launch fanuc_m20ia_35m_planning_demo plan_and_execute.launch.py config:=pid bagfile_path:=<path_to_bag_file>
```
where `<path_to_bag_file>` is the path to the bag file containing the trajectory to be planned.

### Planning with task space control on simulated hardware
First we need to launch the simulated hardware together with the task space controller. This can be achieved by executing the following command:
```bash
ros2 launch acg_resources_fanuc_m20ia_35m_controllers gazebo_ros2_control_demo.launch.py controller_list:=fanuc_m20ia_35m_task_space_controller command_interface:=velocity
```
Then we can launch the planning node with the following command:
```bash
ros2 launch fanuc_m20ia_35m_planning_demo plan_and_execute.launch.py config:=task_space bagfile_path:=<path_to_bag_file>
```
where `<path_to_bag_file>` is the path to the bag file containing the trajectory to be planned.