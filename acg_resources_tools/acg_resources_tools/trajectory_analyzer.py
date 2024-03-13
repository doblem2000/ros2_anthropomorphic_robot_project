"""@package acg_resources_tools.trajectory_analyzer

This module plots the planned and executed trajectories from a bag file, and calculates the Mean Squared Error (MSE) between them. The trajectories can be in joint space or in task space. 

@file trajectory_analyzer.py
@brief Module for analyzing trajectories from a bag file.

This module provides functions for reading messages from a bag file, plotting 3D trajectories, and analyzing trajectories in joint space and task space.

@note This module requires the following packages:
- rosbag2_py
- matplotlib
- argparse
- rclpy
- scipy
- numpy
- control_msgs

@note This module assumes that the bag file contains messages of type control_msgs.action.FollowJointTrajectory_FeedbackMessage for joint space trajectories, and geometry_msgs.msg.PoseStamped for task space trajectories.
"""

import rosbag2_py as rosbag
import matplotlib.pyplot as plt
import argparse
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import control_msgs

import rclpy
from scipy.spatial.transform import Rotation as R
import numpy as np

def read_messages(input_bag: str):
    """
    @brief Read messages from a bag file.

    @param input_bag (str): The path to the bag file.
    
    @return tuple: A tuple containing the topic, message, and timestamp.
    """

    reader = rosbag.SequentialReader()
    reader.open(
        rosbag.StorageOptions(uri=input_bag, storage_id="sqlite3"),
        rosbag.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp
    del reader


def plot_3d_trajectory(trajectory, logger):
    """
    @brief Plot the generated trajectory in a 3D plot.

    @param trajectory: An array that contains the points of planned and executed trajectories.
    @param logger: The logger of the ROS2 node to log the information.
    """

    x_desired = []
    y_desired = []
    z_desired = []
    x_executed = []
    y_executed = []
    z_executed = []
    
    # Loop through each pose in the message
    for point in trajectory:
        
        # Append the x, y, and z coordinates to their respective lists
        x_desired.append(point.feedback.desired.point.pose.position.x)
        y_desired.append(point.feedback.desired.point.pose.position.y)
        z_desired.append(point.feedback.desired.point.pose.position.z)
        x_executed.append(point.feedback.actual.point.pose.position.x)
        y_executed.append(point.feedback.actual.point.pose.position.y)
        z_executed.append(point.feedback.actual.point.pose.position.z)

    # Create a new figure for the orientation
    fig = plt.figure(figsize=[10, 8])
    
    # Add a 3D subplot to the figure for the orientation
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot the orientation using quaternions
    ax.plot(x_desired, y_desired, z_desired, label='Desired')
    ax.plot(x_executed, y_executed, z_executed, label='Executed')

    # Set the labels for the x, y, and z axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Compute the midpoints of the data
    mid_x = (max(x_desired) + min(x_desired)) / 2
    mid_y = (max(y_desired) + min(y_desired)) / 2
    mid_z = (max(z_desired) + min(z_desired)) / 2

    # Compute the maximum range of the data
    max_range = max(max(x_desired) - min(x_desired), max(y_desired) - min(y_desired), max(z_desired) - min(z_desired))

    # Set the limits of the x, y, and z axes to be equal distances from the midpoints
    ax.set_xlim([mid_x - max_range / 2, mid_x + max_range / 2])
    ax.set_ylim([mid_y - max_range / 2, mid_y + max_range / 2])
    ax.set_zlim([mid_z - max_range / 2, mid_z + max_range / 2])

    # Add a legend to the plot
    plt.legend()
    logger.info('3D Trajectories plotted')


def analyze_joint_space_trajectory(trajectory, logger):
    """
    @brief Plot the planned and executed trajectories, and relative velocities, with MSE of a joint space trajectory.

    @param trajectory: An array that contains the points of planned and executed trajectories, and relative velocities.
    @param logger: The logger of the ROS2 node to log the information.
    """

    # Get the joint names
    joint_names = trajectory[0].feedback.joint_names
    
    fig_positions=plt.figure(figsize=[10, 8])
    fig_velocities=plt.figure(figsize=[10, 8])
    
    # Determine the number of elements
    num_elements = len(joint_names)

    # Calculate the grid size
    grid_size = divmod(num_elements, 2)
    grid=fig_positions.add_gridspec(grid_size[0]+grid_size[1],2, hspace=0.6, wspace=0.25)

    i=0 # joint
    j=0 # row
    k=0 # column

    initial_time=trajectory[0].feedback.header.stamp.sec+trajectory[0].feedback.header.stamp.nanosec*pow(10,-9)

    # Loop through each joint
    for joint in joint_names:

        # Initialize the lists
        planned_positions = []
        executed_positions = []
        planned_velocities = []
        executed_velocities = []
        planned_timestamps = []
        executed_timestamps = []
        sum=0

        # Loop through each point in the trajectory
        for point in trajectory:
            
            if not isinstance(point, control_msgs.action._follow_joint_trajectory.FollowJointTrajectory_FeedbackMessage):
                logger.error(f"The trajectory must be of type FollowJointTrajectory_FeedbackMessage")
                raise TypeError("Invalid trajectory type: the trajectory must be of type FollowJointTrajectory_FeedbackMessage")
            
            planned_positions.append(point.feedback.desired.positions[i])
            planned_timestamps.append(point.feedback.header.stamp.sec+point.feedback.header.stamp.nanosec*pow(10,-9)-initial_time)  # Extract the timestamp and convert it to seconds

            executed_positions.append(point.feedback.actual.positions[i])
            executed_timestamps.append(point.feedback.header.stamp.sec+point.feedback.header.stamp.nanosec*pow(10,-9)-initial_time)

            planned_velocities.append(point.feedback.desired.velocities[i])
            executed_velocities.append(point.feedback.actual.velocities[i])

            # Calculate the sum of the squared errors for the MSE
            sum+=point.feedback.error.positions[i]**2

        # Calculate the MSE between the planned and executed trajectories
        mse=sum/len(trajectory)

        # Initialize the subplots
        axis0=fig_positions.add_subplot(grid[j,k])
        axis0.plot(planned_timestamps,planned_positions)
        axis0.plot(executed_timestamps,executed_positions)
        axis0.set_title(joint + " - mse: " + str(round(mse,8)))
        axis0.set_ylabel('Position (rad)')
        axis0.set_xlabel('Time (s)')
        axis0.grid(True)
        axis0.legend(['Planned', 'Executed'])

        axis1=fig_velocities.add_subplot(grid[j,k])
        axis1.plot(planned_timestamps,planned_velocities)
        axis1.plot(executed_timestamps,executed_velocities)
        axis1.set_title(joint)
        axis1.set_ylabel('Velocity (rad/s)')
        axis1.set_xlabel('Time (s)')
        axis1.grid(True)
        axis1.legend(['Planned', 'Executed'])
            
        if k==1:
            j+=1
            k=0
        else:
            k+=1
        
        i+=1
    fig_positions.subplots_adjust(left=0.075, right=0.925, bottom=0.06, top=0.95) # Adjust the spacing between plots
    fig_velocities.subplots_adjust(left=0.075, right=0.925, bottom=0.06, top=0.95) # Adjust the spacing between plots
    logger.info('Trajectories plotted')
    logger.info('Velocities plotted')


def analyze_task_space_trajectory(trajectory, logger):
    """
    @brief Plot the planned and executed trajectories, and relative velocities, with MSE of a task space trajectory.

    @param trajectory: An array that contains the points of planned and executed trajectories, and relative velocities.
    @param logger: The logger of the ROS2 node to log the information.
    """

    fig_positions=plt.figure(figsize=[10, 8])
    fig_velocities=plt.figure(figsize=[10, 8])

    # Define the names of the axes
    name = ['Position x', 'Position y', 'Position z', 'Orientation x', 'Orientation y', 'Orientation z']
    name_velocity = ['Linear velocity x', 'Linear velocity y', 'Linear velocity z', 'Angular velocity x', 'Angular velocity y', 'Angular velocity z']

    # Initialize the lists
    planned_poses=[[],[],[],[],[],[]]    # [x,y,z,roll,pitch,yaw]
    executed_poses=[[],[],[],[],[],[]]   # [x,y,z,roll,pitch,yaw]
    planned_velocities=[[],[],[],[],[],[]]  # [linear_x,linear_y,linear_z,angular_x,angular_y,angular_z]
    executed_velocities=[[],[],[],[],[],[]] # [linear_x,linear_y,linear_z,angular_x,angular_y,angular_z]

    timestamps = []
    sums = []
    mse = []

    k=0 # row
    j=0 # column

    initial_time=trajectory[0].feedback.header.stamp.sec+trajectory[0].feedback.header.stamp.nanosec*pow(10,-9)

    grid1=fig_positions.add_gridspec(3,2, hspace=0.6, wspace=0.25)
    grid2=fig_velocities.add_gridspec(3,2, hspace=0.6, wspace=0.25)

    for i in range(0, 6):
        sums.append(0)

    # Loop through each point in the trajectory
    for point in trajectory:

        planned_poses[0].append(point.feedback.desired.point.pose.position.x)
        planned_poses[1].append(point.feedback.desired.point.pose.position.y)
        planned_poses[2].append(point.feedback.desired.point.pose.position.z)

        if point.feedback.desired.point.pose.orientation.w < 0:
            r=R.from_quat([-point.feedback.desired.point.pose.orientation.x, -point.feedback.desired.point.pose.orientation.y, -point.feedback.desired.point.pose.orientation.z, -point.feedback.desired.point.pose.orientation.w])
        else:
            r=R.from_quat([point.feedback.desired.point.pose.orientation.x, point.feedback.desired.point.pose.orientation.y, point.feedback.desired.point.pose.orientation.z, point.feedback.desired.point.pose.orientation.w])

        [z,y,x]=r.as_euler('zyx', degrees=True)
        # [z,y,x]=get_rpy(r.as_matrix())

        # Calculate the norm of the quaternion to verify that it is unitary 
        # norm=np.sqrt(point.feedback.actual.point.pose.orientation.x**2+point.feedback.actual.point.pose.orientation.y**2+point.feedback.actual.point.pose.orientation.z**2+point.feedback.actual.point.pose.orientation.w**2)
        # print("norm: ", norm)

        planned_poses[3].append(x)
        planned_poses[4].append(y)
        planned_poses[5].append(z)

        executed_poses[0].append(point.feedback.actual.point.pose.position.x)
        executed_poses[1].append(point.feedback.actual.point.pose.position.y)
        executed_poses[2].append(point.feedback.actual.point.pose.position.z)

        if point.feedback.actual.point.pose.orientation.w < 0:
            r=R.from_quat([-point.feedback.actual.point.pose.orientation.x, -point.feedback.actual.point.pose.orientation.y, -point.feedback.actual.point.pose.orientation.z, -point.feedback.actual.point.pose.orientation.w])
        else:
            r=R.from_quat([point.feedback.actual.point.pose.orientation.x, point.feedback.actual.point.pose.orientation.y, point.feedback.actual.point.pose.orientation.z, point.feedback.actual.point.pose.orientation.w])
        
        [z,y,x]=r.as_euler('zyx', degrees=True)
        # [z,y,x]=get_rpy(r.as_matrix())

        executed_poses[3].append(x)
        executed_poses[4].append(y)
        executed_poses[5].append(z)

        planned_velocities[0].append(point.feedback.desired.point.velocity.linear.x)
        planned_velocities[1].append(point.feedback.desired.point.velocity.linear.y)
        planned_velocities[2].append(point.feedback.desired.point.velocity.linear.z)
        planned_velocities[3].append(point.feedback.desired.point.velocity.angular.x)
        planned_velocities[4].append(point.feedback.desired.point.velocity.angular.y)
        planned_velocities[5].append(point.feedback.desired.point.velocity.angular.z)

        executed_velocities[0].append(point.feedback.actual.point.velocity.linear.x)
        executed_velocities[1].append(point.feedback.actual.point.velocity.linear.y)
        executed_velocities[2].append(point.feedback.actual.point.velocity.linear.z)
        executed_velocities[3].append(point.feedback.actual.point.velocity.angular.x)
        executed_velocities[4].append(point.feedback.actual.point.velocity.angular.y)
        executed_velocities[5].append(point.feedback.actual.point.velocity.angular.z)

        timestamps.append(point.feedback.header.stamp.sec+point.feedback.header.stamp.nanosec*pow(10,-9)-initial_time)

        r=R.from_quat([point.feedback.error.point.pose.orientation.x, point.feedback.error.point.pose.orientation.y, point.feedback.error.point.pose.orientation.z, point.feedback.error.point.pose.orientation.w])
        [z,y,x]=r.as_euler('zyx', degrees=True)
        # [z,y,x]=get_rpy(r.as_matrix())

        sums[0]+=point.feedback.error.point.pose.position.x**2
        sums[1]+=point.feedback.error.point.pose.position.y**2
        sums[2]+=point.feedback.error.point.pose.position.z**2
        sums[3]+=x**2
        sums[4]+=y**2
        sums[5]+=z**2
    
    for i in range(0, 6):
        mse.append(sums[i]/len(trajectory))
    
    for i in range(0, 6):
        axis0=fig_positions.add_subplot(grid1[k,j])
        axis0.plot(timestamps,planned_poses[i])
        axis0.plot(timestamps,executed_poses[i])
        axis0.set_title(name[i] + ' - mse: ' + str(round(mse[i],8)))
        if i<3:
            axis0.set_ylabel('Position (m)')
        else:
            axis0.set_ylabel('Orientation (deg)')
        axis0.set_xlabel('Time (s)')
        axis0.grid(True)
        axis0.legend(['Planned', 'Executed'])

        axis1=fig_velocities.add_subplot(grid2[k,j])
        axis1.plot(timestamps,planned_velocities[i])
        axis1.plot(timestamps,executed_velocities[i])
        axis1.set_title(name_velocity[i])
        if i<3:
            axis1.set_ylabel('Velocity (m/s)')
        else:
            axis1.set_ylabel('Velocity (deg/s)')
        axis1.set_xlabel('Time (s)')
        axis1.grid(True)
        axis1.legend(['Planned', 'Executed'])

        if j==1:
            j=0
            k+=1
        else:
            j+=1
    
    fig_positions.subplots_adjust(left=0.075, right=0.925, bottom=0.06, top=0.95) # Adjust the spacing between plots
    fig_velocities.subplots_adjust(left=0.075, right=0.925, bottom=0.06, top=0.95) # Adjust the spacing between plots
    logger.info('Trajectories plotted')
    logger.info('Velocities plotted')


# def get_rpy(matrix):
#     """
#     Get the roll, pitch, and yaw angles from a rotation matrix.

#     Args:
#         matrix: The rotation matrix.

#     Returns:
#         tuple: The roll, pitch, and yaw angles.
#     """

#     # Calculate the roll, pitch, and yaw angles
#     roll = np.arctan2(matrix[1, 0], matrix[0, 0])
#     pitch = np.arctan2(-matrix[2, 0], np.sqrt(matrix[2, 1]**2 + matrix[2, 2]**2))
#     yaw = np.arctan2(matrix[2, 1], matrix[2, 2])

#     return roll, pitch, yaw

def main():
    """
    The main function that executes the program.
    """
    rclpy.init()
    node=rclpy.create_node('trajectory_analyzer')   # Create a ROS2 node for logging
    logger = node.get_logger()
    logger.info('Starting trajectory analyzer')

    # Create the parser
    parser = argparse.ArgumentParser(description='Plot trajectories from a bag file.')

    # Add the arguments
    parser.add_argument('TrajectoriesBagFile', metavar='trajectory_bagfile', type=str, help='The bag file that contains both the planned and the executed trajectories')
    parser.add_argument('--topic', type=str, default=None, help='The topic of the messages containing the trajectory to plot')
    parser.add_argument('--task-space', action='store_true', help='Analyze a task space trajectory instead of a joint space trajectory')

    # Parse the arguments
    args = parser.parse_args()

    # Path to the bag files
    trajectories_bag_file = args.TrajectoriesBagFile
    trajectory_topic = args.topic
    
    ### Read the bag file
    trajectory = []
    topics = []

    # Loop through each message in the trajectories bag file
    for topic, msg, timestamp in read_messages(trajectories_bag_file):
        if topic not in topics:
            topics.append(topic)
        if trajectory_topic is None or topic == trajectory_topic:
            trajectory.append(msg)

    logger.info('Trajectories read')

    if trajectory_topic is None:
        logger.error('No topic specified. Use --topic to specify the topic of the trajectory to plot and select one of the following topics:')
        print(topics)
        exit()
    
    if args.task_space:
        # Analyze task space trajectory
        analyze_task_space_trajectory(trajectory, logger)
        plot_3d_trajectory(trajectory, logger)
    else:
        # Analyze joint space trajectory
        analyze_joint_space_trajectory(trajectory, logger)

    plt.show()

if __name__ == "__main__":
    main()
