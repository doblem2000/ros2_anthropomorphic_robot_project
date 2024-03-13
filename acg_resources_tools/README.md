# acg_resources_tools

This package contains the trajectory analysis tool. This tool plots the planned and executed trajectories from a bag file, and calculates the Mean Squared Error (MSE) between them. The trajectories can be in joint space or in task space. 

## Installing the dependencies
 
In order to install the needed dependencies run the following command inside the project's root folder:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Usage

To run the trajectory analyzer, use the following command:

```bash
ros2 run acg_resources_tools trajectory_analyzer [-h] [--topic TOPIC] [--task-space] path_to_bagfile
```
where `path_to_bagfile` is the path of the bag file that contains the executed and planned trajectories, while the optional parameter `task-space` is used when you want to analyze a task space trajectory. The `topic` parameter is used to specify the topic contained in the bag file that contains the trajectory you want to plot, for example, the sine wave trajectory or the trajectory that takes the robot from the home position to the initial point of the sine wave. In case the `topic` parameter is not specified, the tool throws an error message and shows the list of topics contained in the bag file.

For the joint space the messages contained in the bag file, whatever the trajectory type as long as it is in the joint space, must be: `control_msgs/action/FollowJointTrajectory_FeedbackMessage`. Instead for the task space the messages contained in the bag file must be: `cartesian_control_msgs/action/FollowCartesianTrajectory_FeedbackMessage`.

Two examples of tool use are examined below, using two trajectories made by the Fanuc M-20iA/35M robot, going on to explore the cases of joint space and task space.

### Joint Space
The tool generates two separate windows. In the first window, seven graphs are displayed, each plotting the positions (references and tracking) of the joints in sexagesimal degrees. For each graph, the Mean Squared Error (MSE) is also represented. In the second window, graphs similar to those in the first window are displayed, but for velocities instead of positions.

```bash
ros2 run acg_resources_tools trajectory_analyzer ./src/unisa_acg_ros2/acg_resources_tools/resource/executed_joint_space_trajectories/executed_trajectory_2024-02-09_17-00-16/ --topic '/fanuc_m20ia_cartesian_feedback'
```

For this specific example, the possible topics are: ['/home_trajectory_feedback', '/point2point_feedback', '/fanuc_m20ia_cartesian_feedback'].
The first topic, '/home_trajectory_feedback', contains the path that the robot follows to move from its current position to the home position. The second topic, '/point2point_feedback', represents the path from the home position to the first point of the sine wave. The third topic, '/fanuc_m20ia_cartesian_feedback', is solely the Cartesian path, which represents the sine wave.


### Task Space
The tool generates three separate windows. In the first window contains six graphs. Each graph plots the positions expressed as the x, y, and z components of the end effector, as well as the orientations as x, y, and z orientations of the end effector. Both reference and tracking are plotted on each graph, and the Mean Squared Error (MSE) is calculated for each. In the second window, graphs similar to those in the first window are displayed, but for velocities instead of positions. Finally, in the third window, the three-dimensional trajectory is visualized.

```bash
 ros2 run acg_resources_tools trajectory_analyzer --task-space ./src/unisa_acg_ros2/acg_resources_tools/resource/executed_task_space_trajectories/executed_trajectory_2024-02-13_16-40-29  --topic '/fanuc_m20ia_cartesian_feedback'
```

For this specific example, the possible topics are: ['/home_trajectory_feedback', '/point2point_feedback', '/fanuc_m20ia_cartesian_feedback'].
The first topic, '/home_trajectory_feedback', contains the path that the robot follows to move from its current position to the home position. The second topic, '/point2point_feedback', represents the path from the home position to the first point of the sine wave. The third topic, '/fanuc_m20ia_cartesian_feedback', is solely the Cartesian path, which represents the sine wave.


## Sample trajectories
In the `resource` folder there are two folders `executed_joint_space_trajectories` and `executed_task_space_trajectories` in which sample trajectories can be found for analysis.