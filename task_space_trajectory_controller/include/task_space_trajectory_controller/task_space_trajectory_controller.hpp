#ifndef TASK_SPACE_TRAJECTORY_CONTROLLER_H_INCLUDED
#define TASK_SPACE_TRAJECTORY_CONTROLLER_H_INCLUDED

#include <chrono>
#include <controller_interface/controller_interface.hpp>
#include "cartesian_control_msgs/action/follow_cartesian_trajectory.hpp"
#include "eigen3/Eigen/Core"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/cartesian_trajectory.hpp"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "realtime_tools/realtime_server_goal_handle.h"
#include "tf2_ros/transform_broadcaster.h"

#include "task_space_trajectory_controller_parameters.hpp"
#include "task_space_trajectory_controller/task_space_trajectory_interpolator.hpp"

using namespace std::chrono_literals;

namespace task_space_trajectory_controller
{

using FollowCartesianTrajectoryAction = cartesian_control_msgs::action::FollowCartesianTrajectory;

enum class TaskSpaceTrajectoryStatus : int8_t
{
  IDLE = 0, // Non RT threads can write to command_buffer_, rt_active_goal_ and current_trajectory_ 
  TRACKING_TRAJECTORY = 1 // Ownerships of command_buffer_, rt_active_goal_ and current_trajectory_ is given to update() function
};

class TaskSpaceTrajectoryController : public controller_interface::ControllerInterface
{
public:
  /**
   * @brief Default constructor.
   */
  TaskSpaceTrajectoryController();

  /**
   * @brief Virtual destructor.
   */
  virtual ~TaskSpaceTrajectoryController() = default;

  /**
   * @brief Callback function called during the initialization phase of the controller.
   * @return The callback return value.
   */
  virtual LifecycleNodeInterface::CallbackReturn on_init() override;

  /**
   * @brief Returns the configuration of the command interface.
   * @return The interface configuration.
   */
  virtual controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief Returns the configuration of the state interface.
   * @return The interface configuration.
   */
  virtual controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief Callback function called during the configuration phase of the controller.
   * @param[in] previous_state The previous state of the controller.
   * @return The callback return value.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Callback function called during the activation phase of the controller.
   * @param[in] previous_state The previous state of the controller.
   * @return The callback return value.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Callback function called during the deactivation phase of the controller.
   * @param[in] previous_state The previous state of the controller.
   * @return The callback return value.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Update function called during each control cycle of the controller.
   * @param[in] time The current time.
   * @param[in] period The control period.
   * @return The return type of the update function.
   */
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  /**
   * @brief Reads data from the hardware state interfaces and populates the 'actual_*' member variables.
   */
  void read_state_interfaces();

  /**
   * @brief Sets the 'reference_*' member variables from the given trajectory point.
   * @param[in] command_msg The trajectory point to set the reference from.
   */
  void set_reference_from_trajectory_point(moveit_msgs::msg::CartesianTrajectoryPoint &command_msg);

  /**
   * @brief Generates a trajectory point from the current state of the controller with zero velocity.
   *        This can be used as a reference point for the controller to hold the current position.
   * @return The generated trajectory point.
   */
  std::shared_ptr<moveit_msgs::msg::CartesianTrajectoryPoint> get_holding_position();

  /**
   * @brief Publishes debug information about the current state of the controller to RViz.
   */
  void publish_tfs(Eigen::VectorXd &error);

  /**
   * @brief Gets the current status of the controller.
   * @return The current status of the controller.
   */
  inline TaskSpaceTrajectoryStatus get_controller_status();

  /**
   * @brief Converts Eigen vectors and quaternion to a CartesianPoint message.
   * @param[in] position The position vector.
   * @param[in] orientation The orientation quaternion.
   * @param[in] velocity The velocity vector.
   * @param[out] point The CartesianPoint message to populate.
   */
  inline void eigen_to_point(Eigen::VectorXd &position, Eigen::Quaterniond &orientation, Eigen::VectorXd &velocity, moveit_msgs::msg::CartesianPoint &point);

  /**
   * @brief Sends feedback for the goal.
   * @param time The current time.
   */
  inline void goal_send_feedback(const rclcpp::Time & time);

  /**
   * @brief Callback function called when a goal is received by the action server.
   * @param[in] uuid The UUID of the goal.
   * @param[in] goal The goal.
   * @return The goal response.
   */
  rclcpp_action::GoalResponse goal_received_callback(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowCartesianTrajectoryAction::Goal> goal);

  /**
   * @brief Callback function called when a goal is cancelled by the action server.
   * @param[in] goal_handle The goal handle.
   * @return The cancel response.
   */
  rclcpp_action::CancelResponse goal_cancelled_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowCartesianTrajectoryAction>> goal_handle);

  /**
   * @brief Callback function called when a goal is accepted by the action server.
   * @param[in] goal_handle The goal handle.
   */
  void goal_accepted_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowCartesianTrajectoryAction>> goal_handle);

  /**
   * @brief Callback function called when a command is received on the command topic.
   * @param[in] msg The received command message.
   */
  void command_callback(const std::shared_ptr<moveit_msgs::msg::CartesianTrajectoryPoint> msg);

  /**
   * @brief Validates a trajectory message.
   * @param[in] msg The trajectory message to validate.
   * @return True if the trajectory is valid, false otherwise.
   */
  bool validate_trajectory_msg(const moveit_msgs::msg::CartesianTrajectory & msg);

  inline void update_dynamic_params();
  inline double manipulability_measure(Eigen::MatrixXd &jacobian);
  inline double manipulability_measure_derivative(int joint_index);

  // ROS2 node state
  bool is_initialized = {false}; /**< Flag indicating if the node is initialized. */
  bool is_configured = {false}; /**< Flag indicating if the node is configured. */
  bool is_active = {false}; /**< Flag indicating if the node is active. */
  realtime_tools::RealtimeBuffer<TaskSpaceTrajectoryStatus> rt_controller_status_ = realtime_tools::RealtimeBuffer<TaskSpaceTrajectoryStatus>(TaskSpaceTrajectoryStatus::IDLE); /**< Realtime buffer for the controller status. */

  // ROS2 node parameters
  std::shared_ptr<ParamListener> param_listener_; /**< Pointer to the parameter listener. */
  Params ros_params_; /**< ROS parameters. */
  std::vector<std::string> joint_names; /**< Names of the robot's joints. */
  Eigen::VectorXd joint_default_values; /**< Default values for the robot's joints. */
  Eigen::VectorXd joint_lower_limits; /**< Lower limits for the robot's joints. */
  Eigen::VectorXd joint_upper_limits; /**< Upper limits for the robot's joints. */
  Eigen::VectorXd gains; /**< Gains for the controller. */
  Eigen::VectorXd gains_ff; /**< Feedforward gains for the controller. */

  // Robot state representation
  std::shared_ptr<const moveit::core::RobotModel> kinematic_model_; /**< Pointer to the robot model. */
  std::shared_ptr<moveit::core::RobotState> kinematic_state_; /**< Pointer to the robot state. */
  const moveit::core::JointModelGroup *joint_model_group_; /**< Pointer to the joint model group. */
  Eigen::VectorXd actual_joint_positions_; /**< Actual joint positions. */
  Eigen::VectorXd actual_joint_velocities_; /**< Actual joint velocities. */
  Eigen::VectorXd actual_ee_position_; /**< Actual end effector position. */
  Eigen::Quaterniond actual_ee_orientation_; /**< Actual end effector orientation. */
  Eigen::VectorXd actual_ee_velocity_; /**< Actual end effector velocity. */
  Eigen::MatrixXd actual_jacobian_; /**< Actual Jacobian matrix. */

  // References
  Eigen::VectorXd reference_ee_position_; /**< Reference end effector position. */
  Eigen::Quaterniond reference_ee_orientation_; /**< Reference end effector orientation. */
  Eigen::VectorXd reference_ee_velocity_; /**< Reference end effector velocity. */

  // Hardware interfaces
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_command_velocity_handles; /**< Command velocity handles for the robot's joints. */
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_state_position_handles; /**< State position handles for the robot's joints. */
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_state_velocity_handles; /**< State velocity handles for the robot's joints. */

  // Action server and command topic
  rclcpp_action::Server<FollowCartesianTrajectoryAction>::SharedPtr action_server_; /**< Action server for the Cartesian trajectory. */
  rclcpp::Subscription<moveit_msgs::msg::CartesianTrajectoryPoint>::SharedPtr command_subscriber_; /**< Subscriber for the command topic. */
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; /**< Transform broadcaster for RViz visualization. */
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

  // Realtime buffers
  using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowCartesianTrajectoryAction>;
  using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
  using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr>;
  RealtimeGoalHandleBuffer rt_active_goal_; /**< Realtime buffer for the active goal handle. */
  rclcpp::TimerBase::SharedPtr goal_handle_timer_; /**< Timer for monitoring the action goal handle. */
  rclcpp::Duration action_monitor_period_ = rclcpp::Duration(50ms); /**< Period for monitoring the action goal handle. */
  realtime_tools::RealtimeBuffer<std::shared_ptr<moveit_msgs::msg::CartesianTrajectoryPoint>> command_buffer_; /**< Realtime buffer for the command trajectory point. */
  std::shared_ptr<FollowCartesianTrajectoryAction::Feedback> goal_feedback_; /**< Feedback for the action goal. */

  std::shared_ptr<TaskSpaceTrajectoryInterpolator> trajectory_interpolator = nullptr; /**< Pointer to the trajectory interpolator. Owned by the update() function. */
  realtime_tools::RealtimeBuffer<std::shared_ptr<moveit_msgs::msg::CartesianTrajectory>> current_trajectory_; /**< Realtime buffer for the current trajectory. */
};

}  // namespace task_space_trajectory_controller

#endif
