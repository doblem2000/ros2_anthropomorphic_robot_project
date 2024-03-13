#ifndef PLANNING_NODE_HPP_
#define PLANNING_NODE_HPP_

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include "rosbag2_cpp/reader.hpp"
#include <rosbag2_cpp/writer.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "cartesian_control_msgs/action/follow_cartesian_trajectory.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "eigen3/Eigen/Core"
#include "tf2_eigen/tf2_eigen.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>

#include "planning_node_parameters.hpp"

namespace planning_node
{
/*! 
 * \brief The PlanningNode class is a ROS2 node that provides the functionality to plan and execute trajectories in joint and task space.
 * It is also capable of writing trajectories to and from a bag file.
 * 
*/
class PlanningNode : public rclcpp::Node
{
public:
    using SendTaskSpaceTrajectory = cartesian_control_msgs::action::FollowCartesianTrajectory;
    using GoalHandleSendTaskSpaceTrajectory = rclcpp_action::ClientGoalHandle<SendTaskSpaceTrajectory>;

    using SendJointSpaceTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleSendJointSpaceTrajectory = rclcpp_action::ClientGoalHandle<SendJointSpaceTrajectory>;

    /*! 
     * \brief Construct a new PlanningNode object.
     * @param node_name: the name of the node
     *
    */
    explicit PlanningNode(const std::string node_name);

    /*! Send a task space trajectory to the action server.
     * @param trajectory: the trajectory to be sent
     * @return a shared future to the result of the action
     *
    */
    std::shared_future<rclcpp_action::ClientGoalHandle<PlanningNode::SendTaskSpaceTrajectory>::WrappedResult> task_space_send_goal(const moveit_msgs::msg::CartesianTrajectory trajectory);

    /*! Send a joint space trajectory to the action server.
     * @param trajectory: the trajectory to be sent
     * @return a shared future to the result of the action
     *
    */
    std::shared_future<rclcpp_action::ClientGoalHandle<PlanningNode::SendJointSpaceTrajectory>::WrappedResult> joint_space_send_goal(const trajectory_msgs::msg::JointTrajectory trajectory);

    /*! Generate task space reference for the controller from a joint space trajectory.
     * @param trajectory: the joint space trajectory
     * @param robot_state: the robot state
     * @param logger: the logger object
     * @param target_frame: the target frame
     * @param frame_id: the frame id
     * @param planning_group: the name of the planning group
     * @return the task space trajectory as CartesianTrajectory
     *
    */
    moveit_msgs::msg::CartesianTrajectory get_task_space_trajectory(moveit_msgs::msg::RobotTrajectory trajectory, moveit::core::RobotState &robot_state, const rclcpp::Logger *logger, const std::string target_frame, std::string frame_id, std::string planning_group);
    
    /*! Plan the cartesian path by interpolating between the waypoints.
     * @param waypoints: the vector of waypoints
     * @param logger: the logger object
     * @param move_group_interface: the MoveGroupInterface object
     * @param constraints: the constraints
     * @return a pair containing the percentage of trajectory properly planned and the planned trajectory
     *
    */
    std::pair<double, moveit::planning_interface::MoveGroupInterface::Plan> cartesian_path_planning(std::vector<geometry_msgs::msg::Pose> waypoints, const rclcpp::Logger *logger, moveit::planning_interface::MoveGroupInterface *move_group_interface, moveit_msgs::msg::Constraints &constraints);

    /*! Validate the point to point planned trajectory.
     * @param plan: the planned trajectory
     * @param desired_position: the desired position
     * @param robot_state: the robot state
     * @param tolerance: the tolerance between the desired target position and the planned target position
     * @param logger: the logger object
     * @param target_frame: the target frame
     * @return true if the trajectory is valid, false otherwise
     *
    */
    bool plan_validation(moveit::planning_interface::MoveGroupInterface::Plan plan, geometry_msgs::msg::Pose desired_position, moveit::core::RobotState &robot_state, double tolerance, const rclcpp::Logger *logger, const std::string target_frame);

    /*! Send the trajectory to the proper action server based on the trajectory's type and wait for the result.
     * @param trajectory: the trajectory to be sent
     * @param action_type: the type of the action
     * @return the result of the action
     *
    */
    template <typename TrajectoryType>
    auto execute_trajectory(const TrajectoryType &trajectory) {
        if constexpr (std::is_same<TrajectoryType, trajectory_msgs::msg::JointTrajectory>::value) {
            auto future = this->joint_space_send_goal(trajectory);
            return future;
        } else if constexpr (std::is_same<TrajectoryType, moveit_msgs::msg::CartesianTrajectory>::value) {
            auto future = this->task_space_send_goal(trajectory);
            return future;
        } 
    }

private:
    std::shared_ptr<planning_node::ParamListener> param_listener_;

    rclcpp_action::Client<SendTaskSpaceTrajectory>::SharedPtr task_space_client_ptr_;
    rclcpp_action::Client<SendJointSpaceTrajectory>::SharedPtr joint_space_client_ptr_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    std::string recorded_trajectory_file_name_;

    std::string recorded_trajectory_home_pose_topic_;
    std::string recorded_trajectory_point2point_topic_;
    std::string recorded_trajectory_cartesian_topic_;

    int feedback_sampling_period_;
    int feedback_count_;

    moveit_msgs::msg::CartesianTrajectory trajectory_;
    std::string action_name_;
    std::string action_type_;

    /*! Response callback for the task space action server.
     * @param goal_handle: the goal handle
     *
    */
    void task_space_goal_response_callback(const GoalHandleSendTaskSpaceTrajectory::SharedPtr &goal_handle);

    /*! Feedback callback for the task space action server in which the executed trajectory is recorded in the bag file.
     * @param goal_handle: the goal handle
     * @param feedback: the feedback
     *
    */
    void task_space_feedback_callback(GoalHandleSendTaskSpaceTrajectory::SharedPtr, const std::shared_ptr<const cartesian_control_msgs::action::FollowCartesianTrajectory::Feedback> feedback);

    /*! Result callback for the task space action server.
     * @param result: the result
     *
    */
    void task_space_result_callback(const GoalHandleSendTaskSpaceTrajectory::WrappedResult &result);

    /*! Response callback for the joint space action server.
     * @param goal_handle: the goal handle
     *  
    */
    void joint_space_goal_response_callback(const GoalHandleSendJointSpaceTrajectory::SharedPtr &goal_handle);

    /*! Feedback callback for the joint space action server in which the executed trajectory is recorded in the bag file.
     * @param goal_handle: the goal handle
     * @param feedback: the feedback
     *
    */
    void joint_space_feedback_callback(GoalHandleSendJointSpaceTrajectory::SharedPtr, const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback);

    /*!  Result callback for the joint space action server.
     * @param result: the result
     *
    */
    void joint_space_result_callback(const GoalHandleSendJointSpaceTrajectory::WrappedResult &result);

    /*! Set the name of the recorded trajectory file.
     * @param recorded_trajectory_file_name: the name of the recorded trajectory file
     *
    */
    void set_recorded_trajectory_file_name(std::string recorded_trajectory_file_name);

    /*! Compute direct kinematic through RobotState API.
     * @param robot_state: the robot state
     * @param target_frame: the target frame
     * @param joint_names: the joint names
     * @param positions: the joint positions
     * @return the pose
     *
    */
    geometry_msgs::msg::Pose direct_kinematic(moveit::core::RobotState &robot_state,const std::string target_frame, std::vector<std::string> joint_names, trajectory_msgs::msg::JointTrajectoryPoint::_positions_type positions);

    /*! Compute first order kinematic through RobotState API.
     * @param robot_state: the robot state
     * @param joint_names: the joint names
     * @param positions: the joint positions
     * @param velocities: the joint velocities
     * @param planning_group: the name of the planning group
     * @return the twist
     *
    */
    geometry_msgs::msg::Twist first_order_kinematic(moveit::core::RobotState &robot_state, std::vector<std::string> joint_names, trajectory_msgs::msg::JointTrajectoryPoint::_positions_type positions, trajectory_msgs::msg::JointTrajectoryPoint::_velocities_type velocities, std::string planning_group);
};

}  // namespace planning_node
#endif  
