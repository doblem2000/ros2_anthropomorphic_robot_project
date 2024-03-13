# task_space_trajectory_controller

## Introduction
The package implements a `ros2_control` controller for controlling the end-effector of a robot in the task space. It expects both a pose and a velocity reference in the task space, and it computes the corresponding joint velocity through the (pseudo-)inverse Jacobian matrix. The controller can be used both for regulating the end-effector pose and for tracking a trajectory in the task space.

## Prerequisites
The controller can be configured to control a Joint Model Group of the robot defined in the SRDF file. The following prerequisites must be satisfied in order to use the controller:
- The controller expects each joint of the group to have `position` and `velocity` state interfaces as well as a `velocity` command interface (as it controls the joint velocities).
- The controller expects to read the URDF description of the robot from the topic specified in the `robot_description` parameter (default: `/robot_description`). This means that a `robot_state_publisher` node must be running and publishing the robot state on the topic specified in the `robot_description` parameter.
- The controller expects to read the SRDF description of the robot from the topic `<robot_description>_semantic` where `<robot_description>` is the value of the `robot_description` parameter. This means that `move_group` must be running and publishing the SRDF description of the robot on the topic `<robot_description>_semantic`.

## Commanding the controller
The controller can be used both for regulation and tracking. The controller has two internal states that reflect these features: `IDLE` and `TRACKING_TRAJECTORY`. When the controller is activated, it starts in the `IDLE` state and holds the current pose of the end-effector. When a trajectory is sent to the controller, it switches to the `TRACKING_TRAJECTORY` state and starts tracking the trajectory. When the trajectory is completed, the controller switches back to the `IDLE` state and holds the current pose of the end-effector.

### Regulating the end-effector pose
In order to regulate the end-effector pose on a given point, the user can send a reference `moveit_msgs/msg/CartesianTrajectoryPoint` on the `~/command` topic of the controller. Doing this is only recommended for testing purposes as the reference of the controller is instantly changed and this can lead to undesired behaviours when the reference is too far from the current pose of the end-effector. When the controller is in `TRACKING_TRAJECTORY` state, the references sent on the `~/command` topic are ignored.

### Tracking a trajectory in the task space
In order to track a trajectory in the task space, the user can use the action `~/FollowCartesianTrajectory`. The message type is `cartesian_control_msgs/action/FollowCartesianTrajectory`. The trajectory is validated before the controller accepts the goal. The trajectory is accepted only if:
- the trajectory is not empty;
- when the fist point of the trajectory has a timestamp equal to zero, the euclidean distance between the current pose of the end-effector and the first point of the trajectory is less than the `first_point_tolerance` parameter of the controller;
- the timestamp of each point of the trajectory is greater than the timestamp of the previous point;
- the header frame of the trajectory is equal to the `robot_base_link` parameter of the controller;
- the controlled frame of the trajectory is equal to the `end_effector_link` parameter of the controller;

If the goal is accepted by the controller, the controller will switch to the `TRACKING_TRAJECTORY` state and will start tracking the trajectory. When the trajectory is completed, the controller will switch back to the `IDLE` state and will hold the current pose of the end-effector. It is possible to cancel the goal at any time. If the goal is cancelled, the controller will switch back to the `IDLE` state and will hold the current pose of the end-effector. However, if a new trajectory is sent to the controller while it is tracking a trajectory, the new trajectory will be rejected.

## Trajectory interpolation
The controller interpolates the given trajectory in the task space. The interpolation is performed dynamically, i.e. the controller computes the trajectory in the task space at each control cycle. 

By default, a linear interpolation is used. For the orientations, SLERP interpolation in the quaternion space is used.

## Controller parameters
The controller parameters are:
- `joint_model_group_name`: the name of the planning group to use;
- `robot_base_link`: robot frame workspace velocities are defined in;
- `end_effector_link`: the end-effector link of the robot. It must be the last link of the kinematic chain of the planning group;
- `robot_description`: the topic where the URDF description of the robot is published;
- `gains`: a dictionary containing the gains for the controller. It includes `gains.position`, `gains.orientation`, `gains.velocity` and `gains.k0` gains. 
    - `gains.position` includes `gains.position.x`, `gains.position.y` and `gains.position.z` gains for the position control;
    - `gains.orientation` includes `gains.orientation.x`, `gains.orientation.y` and `gains.orientation.z` gains for the orientation control;
    - `gains.velocity` includes `gains.velocity.linear.x`, `gains.velocity.linear.y`, `gains.velocity.linear.z` gains for the linear velocity and `gains.velocity.angular.x`, `gains.velocity.angular.y` and `gains.velocity.angular.z` gains for the velocity control;
    - `gains.k0` is the gain for the movements in the null space control.
- `action_server.first_point_tolerance`: the error tolerance for the first point of the trajectory. If the error is greater than this value, the controller will not start the trajectory execution;
- `use_nullspace`: boolean value to enable/disable the null space control;
- `debug`: boolean value to enable/disable the debug mode. This mode publishes TFs for the reference and the current pose of the end-effector on the `/tf` topic.
  
## Installing the dependencies
 
In order to install the needed dependencies run the command

```bash
rosdep install --from-paths src --ignore-src -r -y
```
    
inside the project's root folder

## Running the package
The package has no launch files as those in `controller_manager` package can be used to spawn the controller (i.e. `ros2 launch controller_manager spawner`). However, for ease of use, it is recommended to use the launch files provided in the `acg_resources_fanuc_m20ia_35m_controllers` package. For more info, refer to the README of that package.

## Known issues
- A limitation of the chosen control algorithm is that it is not collision-aware as it is intended to be used in free space. This means that the controller can lead to undesired behaviours when the robot is in self-collision or in collision with the environment. In the case of the Fanuc M20-iA/35M robot, this usually happens when the end-effector collides with the link 5 of the robot. To mitigate this issue, one should use the controller in combination with a collision-aware motion planner, such as MoveIt!, but this does not guarantee that the robot will not self-collide during the trajectory execution.
- Another problem that is common to all kinematic control algorithms is that the controller could impose high joint velocities to the robot when the robot is close to a singularity. We tried to overcome this problem by using the null space to maximize the manipulability measure of the robot, however in order to keep the controller stable we had to use a very low gain for the null space component. This leds to a very slow convergence of the nullspace component (but it does not affect the tracking of the trajectory in the task space). As a consequence, when using the task space controller **one should wait for the robot to stand still** before sending a new trajectory to the controller. Doing so minimizes the risk of the robot to be close to a singularity and allows for better tracking performances.

<!-- In the following we provide an example of how to spawn the controller for the `fanuc_m20ia_35m` robot without a launch file. Each command must be run in a different terminal.

1. Generate the URDF description of the robot by using xacro. The URDF must contain the `mock_components/GenericSystem` ros2_control plugin. In the case of the `fanuc_m20ia_35m` robot, the URDF is generated by using the following command:
    ```bash
    xacro <path_to_xacro_file> use_mock_hardware:=true command_interface:=velocity > <path_to_output_URDF>
    ```
    where the path to the xacro file is `~/acg_resources_fanuc_m20ia_35m_moveit_config/config/fanuc_m20ia_35m.urdf.xacro`.

2. Run the `robot_state_publisher` node passing the URDF description of the robot as argument (eventually generated by xacro):
    ```bash
    ros2 run robot_state_publisher robot_state_publisher <path_to_URDF>
    ```

3. Run the `move_group` node by using the launch file provided in the `fanuc_m20ia_35m_moveit_config` package:
    ```bash
    ros2 launch acg_resources_fanuc_m20ia_35m_moveit_config move_group.launch.py
    ```

4. Run RViz2 and load the configuration file provided in the `fanuc_m20ia_35m_moveit_config` package with the following command:
    ```bash
    ros2 launch acg_resources_fanuc_m20ia_35m_moveit_config moveit_rviz.launch.py
    ```

5. Run the `controller_manager` node by passing a proper configuration file as argument:
    ```bash
    ros2 run controller_manager controller_manager --ros-args --params-file <path_to_configuration_file>
    ```
    An example configuration file is provided in the `config` folder of the `acg_resources_fanuc_m20ia_35m_controllers` package. For more info, refer to the README of that package.

6. Spawn the joint_state_broadcaster by using the `controller_manager` node:
    ```bash
    ros2 run controller_manager spawner joint_state_broadcaster
    ```

7. Spawn the controller by using the `controller_manager` node:
    ```bash
    ros2 run controller_manager spawner fanuc_m20ia_35m_task_space_controller
    ```
    Note that the name of the controller is defined in the configuration file passed to the `controller_manager` node. Now the controller is ready to be used.

8. Send a reference to the controller on the `~/command` topic
    ```bash
    ros2 topic pub -1 /fanuc_m20ia_35m_task_space_controller/command moveit_msgs/msg/CartesianTrajectoryPoint "{\
    point: {\
        pose: {\
        position: {\
            x: 2.5,\
            y: 0.0,\
            z: 0.0\
        },\
        orientation: {\
            x: 0.0,\
            y: 0.0,\
            z: 0.0,\
            w: 0.0\
        }\
        },\
        velocity: {\
        linear: {\
            x: 0.0,\
            y: 0.0,\
            z: 0.0\
        },\
        angular: {\
            x: 0.0,\
            y: 0.0,\
            z: 0.0\
        }\
        },\
    },\
    time_from_start: {sec: 0, nanosec: 0}\
    }"
    ``` -->