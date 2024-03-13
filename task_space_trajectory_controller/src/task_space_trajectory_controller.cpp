#include "task_space_trajectory_controller/task_space_trajectory_controller.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

#include "task_space_trajectory_controller_parameters.hpp"
#include "task_space_trajectory_controller/task_space_trajectory_interpolator.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using FollowCartesianTrajectoryAction = cartesian_control_msgs::action::FollowCartesianTrajectory;
using RealtimeGoalHandle = realtime_tools::RealtimeServerGoalHandle<FollowCartesianTrajectoryAction>;
using RealtimeGoalHandlePtr = std::shared_ptr<RealtimeGoalHandle>;
using RealtimeGoalHandleBuffer = realtime_tools::RealtimeBuffer<RealtimeGoalHandlePtr>;

namespace task_space_trajectory_controller
{
TaskSpaceTrajectoryController::TaskSpaceTrajectoryController() : controller_interface::ControllerInterface() {}

controller_interface::InterfaceConfiguration TaskSpaceTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names.size());
  for (const auto & joint_name : joint_names)
  {
    conf.names.push_back(joint_name + "/position");
    conf.names.push_back(joint_name + "/velocity");
  }
  return conf;
}

controller_interface::InterfaceConfiguration TaskSpaceTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names.size());
  for (const auto & joint_name : joint_names)
  {
    conf.names.push_back(joint_name + "/velocity");
  }
  return conf;
}

CallbackReturn TaskSpaceTrajectoryController::on_init()
{
  RCLCPP_INFO(get_node()->get_logger(), "Initializing controller");
  if (this->is_initialized)
  {
    return CallbackReturn::SUCCESS;
  }
  this->param_listener_ = std::make_shared<ParamListener>(get_node());
  this->is_initialized = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceTrajectoryController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_node()->get_logger(), "Configuring controller");
  if(this->is_configured){
    return CallbackReturn::SUCCESS;
  }
  
  // Read parameters
  this->ros_params_ = this->param_listener_->get_params();

  // Spawn slave node to load robot model
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("robot_model_loader");
  bool keep_slave_alive = true;

  std::thread t([node, &keep_slave_alive, this]() {
    rclcpp::executors::SingleThreadedExecutor executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(node);
    RCLCPP_INFO(this->get_node()->get_logger(), "Spinning up robot model loader node");
    while(keep_slave_alive){
      executor.spin_some(100ms);
    }
    RCLCPP_INFO(this->get_node()->get_logger(), "Robot model loader node stopped spinning");
  });

  // Load robot model
  std::shared_ptr<const robot_model_loader::RobotModelLoader> robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(node,this->ros_params_.robot_description, true);
  this->kinematic_model_ = robot_model_loader->getModel();
  if(this->kinematic_model_ == nullptr){
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load robot model. Check that /robot_description and /robot_description_semantic are correctly set.");
    keep_slave_alive = false;
    t.join();
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Loaded robot model. Killing robot model loader node...");
  keep_slave_alive = false;
  t.join();
  this->kinematic_state_ = std::make_shared<moveit::core::RobotState>(this->kinematic_model_);
  this->joint_model_group_ = this->kinematic_model_->getJointModelGroup(this->ros_params_.joint_model_group_name);

  // Get joint names
  this->joint_names = this->joint_model_group_->getActiveJointModelNames();

  // Print joint names
  std::string joint_names_string = "\n";
  for (const auto & joint_name : this->joint_names)
  {
    joint_names_string += "\t- " + joint_name + "\n";
  }
  RCLCPP_INFO(get_node()->get_logger(), "Loaded joint group %s with %zu joints. Joint names: %s", this->ros_params_.joint_model_group_name.c_str(), this->joint_names.size(), joint_names_string.c_str());

  // Get joint bounds and default positions
  this->joint_lower_limits = Eigen::VectorXd(this->joint_names.size());
  this->joint_upper_limits = Eigen::VectorXd(this->joint_names.size());
  this->joint_default_values = Eigen::VectorXd(this->joint_names.size());
  const moveit::core::JointBoundsVector joint_bounds = this->joint_model_group_->getActiveJointModelsBounds();
  this->joint_model_group_->getVariableDefaultPositions(this->joint_default_values.data());

  // Set joint bounds
  for(size_t i=0; i<joint_bounds.size(); i++){
    const moveit::core::VariableBounds bounds = joint_bounds[i]->at(0);
    RCLCPP_INFO(get_node()->get_logger(), "Joint %s mean value: %f", joint_names.at(i).c_str(), this->joint_default_values(i));
    if(bounds.position_bounded_){
      RCLCPP_INFO(get_node()->get_logger(), "Joint %s bounds: [%f, %f]", joint_names.at(i).c_str(), bounds.min_position_, bounds.max_position_);
      this->joint_lower_limits(i) = bounds.min_position_;
      this->joint_upper_limits(i) = bounds.max_position_;
    }
    else {
      RCLCPP_INFO(get_node()->get_logger(), "Joint %s is unbounded", joint_names.at(i).c_str());
      this->joint_lower_limits(i) = -std::numeric_limits<double>::infinity();
      this->joint_upper_limits(i) = std::numeric_limits<double>::infinity();
    }
  }

  // Initialize state buffers
  this->actual_joint_positions_ = Eigen::VectorXd(this->joint_names.size());
  this->actual_joint_velocities_ = Eigen::VectorXd(this->joint_names.size());
  this->actual_ee_position_ = Eigen::VectorXd(3);
  this->reference_ee_position_ = Eigen::VectorXd(3);
  this->reference_ee_velocity_ = Eigen::VectorXd(6);

  this->gains = Eigen::VectorXd(6);
  this->gains << this->ros_params_.gains.position.x,
                  this->ros_params_.gains.position.y,
                  this->ros_params_.gains.position.z,
                  this->ros_params_.gains.orientation.x,
                  this->ros_params_.gains.orientation.y,
                  this->ros_params_.gains.orientation.z;
  this->gains_ff = Eigen::VectorXd(6);
  this->gains_ff << this->ros_params_.gains.velocity.linear.x,
                  this->ros_params_.gains.velocity.linear.y,
                  this->ros_params_.gains.velocity.linear.z,
                  this->ros_params_.gains.velocity.angular.x,
                  this->ros_params_.gains.velocity.angular.y,
                  this->ros_params_.gains.velocity.angular.z;

  this->current_trajectory_.initRT(nullptr);
  this->rt_controller_status_.initRT(TaskSpaceTrajectoryStatus::IDLE);
  this->rt_active_goal_.initRT(RealtimeGoalHandlePtr());
  this->trajectory_interpolator = std::make_shared<TaskSpaceTrajectoryInterpolator>();
  this->trajectory_interpolator->set_interpolation_method(InterpolationMethod::LINEAR);
  this->goal_feedback_ = std::make_shared<FollowCartesianTrajectoryAction::Feedback>();

  // Configure action server
  this->action_server_ = rclcpp_action::create_server<FollowCartesianTrajectoryAction>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/follow_cartesian_trajectory",
    std::bind(&TaskSpaceTrajectoryController::goal_received_callback, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&TaskSpaceTrajectoryController::goal_cancelled_callback, this, std::placeholders::_1),
    std::bind(&TaskSpaceTrajectoryController::goal_accepted_callback, this, std::placeholders::_1));

  // Configure command topic
  this->command_subscriber_ = get_node()->create_subscription<moveit_msgs::msg::CartesianTrajectoryPoint>(
    std::string(get_node()->get_name()) + "/command", 1, std::bind(&TaskSpaceTrajectoryController::command_callback, this, std::placeholders::_1));

  // Configure tf broadcaster
  if(this->ros_params_.debug){
    RCLCPP_INFO(get_node()->get_logger(), "Debug mode enabled. Publishing TFs");
    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());
    this->marker_publisher_ = get_node()->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 0);
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Debug mode disabled. Not publishing TFs");
  }

  RCLCPP_INFO(get_node()->get_logger(), "Controller configured successfully");
  this->is_configured = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceTrajectoryController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_node()->get_logger(), "Activating controller");
  if(this->is_active){
    return CallbackReturn::SUCCESS;
  }

  if(this->ros_params_.debug){
    this->marker_publisher_->on_activate();
  }

  // Get command handles.
  if (!controller_interface::get_ordered_interfaces(command_interfaces_,
   this->joint_names, hardware_interface::HW_IF_VELOCITY, this->joint_command_velocity_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu velocity command interfaces, got %zu.",
                  this->joint_names.size(), this->joint_command_velocity_handles.size());
    return CallbackReturn::ERROR;
  }

  // Get state handles.
  if (!controller_interface::get_ordered_interfaces(state_interfaces_,
   this->joint_names, hardware_interface::HW_IF_POSITION, this->joint_state_position_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu position state interfaces, got %zu.",
                 this->joint_names.size(), this->joint_state_position_handles.size());
    return CallbackReturn::ERROR;
  }
  if (!controller_interface::get_ordered_interfaces(state_interfaces_,
   this->joint_names, hardware_interface::HW_IF_VELOCITY, this->joint_state_velocity_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected %zu velocity state interfaces, got %zu.",
                 this->joint_names.size(), this->joint_state_position_handles.size());
    return CallbackReturn::ERROR;
  }

  // Read state
  read_state_interfaces();

  // Set holding position as reference
  const auto reference_msg = get_holding_position();
  this->command_buffer_.initRT(reference_msg);

  this->is_active = true;
  RCLCPP_INFO(get_node()->get_logger(), "Controller activated successfully");
  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceTrajectoryController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating controller");
  if (this->is_active)
  {
    this->joint_command_velocity_handles.clear();
    this->joint_state_position_handles.clear();
    this->joint_state_velocity_handles.clear();
    this->release_interfaces();
    this->is_active = false;
  }
  return CallbackReturn::SUCCESS;
}

void TaskSpaceTrajectoryController::read_state_interfaces(){
  for (unsigned int i = 0; i < this->joint_names.size(); i++)
  {
    this->actual_joint_positions_(i) = this->joint_state_position_handles[i].get().get_value();
    this->actual_joint_velocities_(i) = this->joint_state_velocity_handles[i].get().get_value();
  }
  this->kinematic_state_->setJointGroupPositions(this->joint_model_group_, this->actual_joint_positions_);
  this->kinematic_state_->update();
  Eigen::Isometry3d actual_ee_isometry = this->kinematic_state_->getGlobalLinkTransform(this->ros_params_.end_effector_link);
  this->actual_ee_position_(0) = actual_ee_isometry.translation().x();
  this->actual_ee_position_(1) = actual_ee_isometry.translation().y();
  this->actual_ee_position_(2) = actual_ee_isometry.translation().z();
  this->actual_ee_orientation_ = Eigen::Quaterniond(actual_ee_isometry.rotation());

  this->actual_jacobian_ = this->kinematic_state_->getJacobian(this->joint_model_group_);
  
  this->actual_ee_velocity_ = this->actual_jacobian_ * this->actual_joint_velocities_;
}

void TaskSpaceTrajectoryController::set_reference_from_trajectory_point(moveit_msgs::msg::CartesianTrajectoryPoint &command_msg){
  this->reference_ee_position_(0) = command_msg.point.pose.position.x;
  this->reference_ee_position_(1) = command_msg.point.pose.position.y;
  this->reference_ee_position_(2) = command_msg.point.pose.position.z;
  this->reference_ee_orientation_ = Eigen::Quaterniond(command_msg.point.pose.orientation.w,
                                                      command_msg.point.pose.orientation.x,
                                                      command_msg.point.pose.orientation.y,
                                                      command_msg.point.pose.orientation.z);
  this->reference_ee_velocity_(0) = command_msg.point.velocity.linear.x;
  this->reference_ee_velocity_(1) = command_msg.point.velocity.linear.y;
  this->reference_ee_velocity_(2) = command_msg.point.velocity.linear.z;
  this->reference_ee_velocity_(3) = command_msg.point.velocity.angular.x;
  this->reference_ee_velocity_(4) = command_msg.point.velocity.angular.y;
  this->reference_ee_velocity_(5) = command_msg.point.velocity.angular.z;  
}

inline void TaskSpaceTrajectoryController::eigen_to_point(Eigen::VectorXd &position, Eigen::Quaterniond &orientation, Eigen::VectorXd &velocity, moveit_msgs::msg::CartesianPoint &point){
  point.pose.position.x = position(0);
  point.pose.position.y = position(1);
  point.pose.position.z = position(2);
  point.pose.orientation.x = orientation.x();
  point.pose.orientation.y = orientation.y();
  point.pose.orientation.z = orientation.z();
  point.pose.orientation.w = orientation.w();
  point.velocity.linear.x = velocity(0);
  point.velocity.linear.y = velocity(1);
  point.velocity.linear.z = velocity(2);
  point.velocity.angular.x = velocity(3);
  point.velocity.angular.y = velocity(4);
  point.velocity.angular.z = velocity(5);
}

inline void TaskSpaceTrajectoryController::update_dynamic_params(){
  if (this->param_listener_->is_old(this->ros_params_)) {
    this->ros_params_ = this->param_listener_->get_params();
    this->gains << this->ros_params_.gains.position.x,
                    this->ros_params_.gains.position.y,
                    this->ros_params_.gains.position.z,
                    this->ros_params_.gains.orientation.x,
                    this->ros_params_.gains.orientation.y,
                    this->ros_params_.gains.orientation.z;
    this->gains_ff << this->ros_params_.gains.velocity.linear.x,
                    this->ros_params_.gains.velocity.linear.y,
                    this->ros_params_.gains.velocity.linear.z,
                    this->ros_params_.gains.velocity.angular.x,
                    this->ros_params_.gains.velocity.angular.y,
                    this->ros_params_.gains.velocity.angular.z;
  }
}

inline void TaskSpaceTrajectoryController::goal_send_feedback(const rclcpp::Time & time){
  this->goal_feedback_->header.stamp = time;
  this->goal_feedback_->header.frame_id = this->ros_params_.robot_base_link;
  this->goal_feedback_->controlled_frame = this->ros_params_.end_effector_link;
  eigen_to_point(this->actual_ee_position_, this->actual_ee_orientation_ , this->actual_ee_velocity_, this->goal_feedback_->actual.point);
  eigen_to_point(this->reference_ee_position_, this->reference_ee_orientation_, this->reference_ee_velocity_, this->goal_feedback_->desired.point);
  Eigen::VectorXd position_error = this->reference_ee_position_ - this->actual_ee_position_;
  Eigen::Quaterniond orientation_error = this->reference_ee_orientation_ * this->actual_ee_orientation_.inverse();
  Eigen::VectorXd velocity_error = this->reference_ee_velocity_ - this->actual_ee_velocity_;
  eigen_to_point(position_error, orientation_error, velocity_error, this->goal_feedback_->error.point);
  rt_active_goal_.readFromRT()->get()->setFeedback(this->goal_feedback_);
}

inline double TaskSpaceTrajectoryController::manipulability_measure(Eigen::MatrixXd &jacobian){
  return sqrt((jacobian*jacobian.transpose()).determinant());
}

inline double TaskSpaceTrajectoryController::manipulability_measure_derivative(int joint_index){
  Eigen::VectorXd joint_selector = Eigen::VectorXd::Zero(this->joint_names.size());
  double epsilon = 1e-4;
  joint_selector(joint_index) = 1;
  this->kinematic_state_->setJointGroupPositions(this->joint_model_group_, this->actual_joint_positions_ + epsilon*joint_selector);
  this->kinematic_state_->update();
  auto jacobian_plus = this->kinematic_state_->getJacobian(this->joint_model_group_);
  this->kinematic_state_->setJointGroupPositions(this->joint_model_group_, this->actual_joint_positions_ - epsilon*joint_selector);
  this->kinematic_state_->update();
  auto jacobian_minus = this->kinematic_state_->getJacobian(this->joint_model_group_);
  return (manipulability_measure(jacobian_plus) - manipulability_measure(jacobian_minus)) / (2*epsilon);
}

controller_interface::return_type TaskSpaceTrajectoryController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if(!this->is_active){
    return controller_interface::return_type::OK;
  }

  // Update dynamic parameters
  update_dynamic_params();

  // Read state
  read_state_interfaces();

  std::shared_ptr<moveit_msgs::msg::CartesianTrajectoryPoint> command_msg = *(this->command_buffer_.readFromRT());
  TrajectoryPointConstIter start_segment_itr, end_segment_itr;

  // Check if the controller is tracking a trajectory
  const auto active_goal = *rt_active_goal_.readFromRT();
  bool valid_point = false;
  bool trajectory_execution_finished = false;
  if(get_controller_status() == TaskSpaceTrajectoryStatus::TRACKING_TRAJECTORY && active_goal){
    // If the interpolator is not initialized with the current trajectory, initialize it
    const auto current_trajectory = *(this->current_trajectory_.readFromRT());
    if(this->trajectory_interpolator->get_trajectory_ptr() != current_trajectory){
      this->trajectory_interpolator->set_trajectory(current_trajectory, get_holding_position());
    }
    // Get the current reference from the interpolator
    // Notice that when TRACKING_TRAJECTORY, the command buffer is owned by the update() function
    valid_point = this->trajectory_interpolator->sample(time, command_msg, start_segment_itr, end_segment_itr, get_node()->get_logger());
    trajectory_execution_finished = end_segment_itr == this->trajectory_interpolator->end();
    // If the trajectory is finished, set the controller status to IDLE and stop tracking the trajectory
    if(valid_point && trajectory_execution_finished){
      RCLCPP_INFO(get_node()->get_logger(), "Finished tracking trajectory");
      // Set goal as succeeded
      std::shared_ptr<FollowCartesianTrajectoryAction::Result> action_res = std::make_shared<FollowCartesianTrajectoryAction::Result>();
      action_res->error_code = FollowCartesianTrajectoryAction::Result::SUCCESS;
      active_goal->setSucceeded(action_res);
      // Reset goal handle
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
      // Set controller status to IDLE
      this->rt_controller_status_.reset();
      this->rt_controller_status_.initRT(TaskSpaceTrajectoryStatus::IDLE);
      // Set holding position from the last point of the trajectory
      command_msg->point.velocity.linear.x = 0.0;
      command_msg->point.velocity.linear.y = 0.0;
      command_msg->point.velocity.linear.z = 0.0;
      command_msg->point.velocity.angular.x = 0.0;
      command_msg->point.velocity.angular.y = 0.0;
      command_msg->point.velocity.angular.z = 0.0;
    }
  }

  // Read command from the command buffer
  set_reference_from_trajectory_point(*command_msg);

  // If executing a trajectory, send feedback
  if(get_controller_status() == TaskSpaceTrajectoryStatus::TRACKING_TRAJECTORY && active_goal && valid_point && !trajectory_execution_finished){
    goal_send_feedback(time);
  }


  // Calculate control law
  Eigen::MatrixXd actual_jacobian_pseudo_inverse = this->actual_jacobian_.completeOrthogonalDecomposition().pseudoInverse();

  Eigen::VectorXd position_error = this->reference_ee_position_ - this->actual_ee_position_;
  Eigen::Quaterniond error_quaternion = this->reference_ee_orientation_ * this->actual_ee_orientation_.inverse();
  //Eigen::VectorXd orientation_error = Eigen::Vector3d(error_quaternion.x(), error_quaternion.y(), error_quaternion.z());
  Eigen::VectorXd orientation_error = Eigen::Vector3d();
  //error_rotation_matrix.getRPY(orientation_error(0), orientation_error(1), orientation_error(2));
  //RCLCPP_INFO(get_node()->get_logger(), "Orientation error: %f, %f, %f", orientation_error(0), orientation_error(1), orientation_error(2));
  if(this->ros_params_.orientation_error_axis_angle){
    Eigen::AngleAxisd error_angle_axis(error_quaternion);
    orientation_error = error_angle_axis.axis() * error_angle_axis.angle();
  } else {
    tf2::Matrix3x3 error_rotation_matrix(tf2::Quaternion(error_quaternion.x(), error_quaternion.y(), error_quaternion.z(), error_quaternion.w()));
    error_rotation_matrix.getRPY(orientation_error(0), orientation_error(1), orientation_error(2));
  }
  Eigen::VectorXd error(6);
  error << position_error, orientation_error;

  Eigen::VectorXd joint_velocities_command = actual_jacobian_pseudo_inverse * (this->gains_ff.asDiagonal() * this->reference_ee_velocity_ + this->gains.asDiagonal() * error);

  if(this->ros_params_.use_nullspace){
    int number_of_joints = this->joint_names.size();
    Eigen::VectorXd q_dot0(number_of_joints);
    Eigen::ArrayXd nabla_w = Eigen::ArrayXd::Zero(number_of_joints);
    if(this->ros_params_.nullspace_objective == "manipulability_measure"){
      if((this->actual_jacobian_*this->actual_jacobian_.transpose()).determinant() < 1e-5){
        q_dot0 = Eigen::ArrayXd::Ones(number_of_joints)*-1.0;
      } else {
        for(int i=0; i<number_of_joints; i++){
          nabla_w(i) = manipulability_measure_derivative(i);
        }
        q_dot0 = this->ros_params_.gains.k0 * nabla_w;
      }
    } else if(this->ros_params_.nullspace_objective == "joint_limits"){
      nabla_w = (-1.0/number_of_joints) * (this->actual_joint_positions_ - this->joint_default_values).array() / (this->joint_upper_limits - this->joint_lower_limits).array().pow(2);
      q_dot0 = this->ros_params_.gains.k0 * nabla_w;
    }
    Eigen::MatrixXd identity_matrix = Eigen::MatrixXd::Identity(number_of_joints, number_of_joints);
    joint_velocities_command += (identity_matrix - actual_jacobian_pseudo_inverse * this->actual_jacobian_) * q_dot0;
  }

  // Print debug info if debug mode is enabled
  if(this->ros_params_.debug){
    publish_tfs(error);
    //RCLCPP_INFO(get_node()->get_logger(), "Manipulability: %f", (this->actual_jacobian_*this->actual_jacobian_.transpose()).determinant());
  }

  // Send the command velocities to the robot
  for (std::size_t i = 0; i < this->joint_names.size(); i++)
    this->joint_command_velocity_handles[i].get().set_value(joint_velocities_command(i));

  return controller_interface::return_type::OK;
}

void TaskSpaceTrajectoryController::publish_tfs(Eigen::VectorXd &error){
  // Publish actual end effector pose
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.transform.translation.x = this->actual_ee_position_(0);
  transform_stamped.transform.translation.y = this->actual_ee_position_(1);
  transform_stamped.transform.translation.z = this->actual_ee_position_(2);
  transform_stamped.transform.rotation.x = this->actual_ee_orientation_.x();
  transform_stamped.transform.rotation.y = this->actual_ee_orientation_.y();
  transform_stamped.transform.rotation.z = this->actual_ee_orientation_.z();
  transform_stamped.transform.rotation.w = this->actual_ee_orientation_.w();
  transform_stamped.header.stamp = get_node()->now();
  transform_stamped.header.frame_id = this->ros_params_.robot_base_link;
  transform_stamped.child_frame_id = this->ros_params_.end_effector_link + "_actual";
  this->tf_broadcaster_->sendTransform(transform_stamped);

  //Publish reference end effector pose
  const std::shared_ptr<moveit_msgs::msg::CartesianTrajectoryPoint> command_msg = *(this->command_buffer_.readFromRT());
  transform_stamped.header.stamp = get_node()->now();
  transform_stamped.header.frame_id = this->ros_params_.robot_base_link;
  transform_stamped.child_frame_id = this->ros_params_.end_effector_link + "_reference";
  transform_stamped.transform.translation.x = command_msg->point.pose.position.x;
  transform_stamped.transform.translation.y = command_msg->point.pose.position.y;
  transform_stamped.transform.translation.z = command_msg->point.pose.position.z;
  transform_stamped.transform.rotation = command_msg->point.pose.orientation;
  this->tf_broadcaster_->sendTransform(transform_stamped);

  // Publish orientation error based on RPY
  Eigen::VectorXd position_error = this->reference_ee_position_ - this->actual_ee_position_;
  Eigen::Quaterniond error_quaternion = this->reference_ee_orientation_ * this->actual_ee_orientation_.inverse();
  Eigen::AngleAxisd error_angle_axis(error_quaternion);
  tf2::Matrix3x3 error_rotation_matrix(tf2::Quaternion(error_quaternion.x(), error_quaternion.y(), error_quaternion.z(), error_quaternion.w()));
  Eigen::Vector3d rpy_error;
  error_rotation_matrix.getRPY(rpy_error(0), rpy_error(1), rpy_error(2));

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = this->ros_params_.robot_base_link;
  marker.header.stamp = get_node()->now();
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.points.resize(2);
  marker.points[0].x = 0.0;
  marker.points[0].y = 0.0;
  marker.points[0].z = 0.0;
  
  marker.points[1].x = rpy_error(0);
  marker.points[1].y = rpy_error(1);
  marker.points[1].z = rpy_error(2);
  marker.scale.x = 0.05;
  marker.color.a = 0.5;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  this->marker_publisher_->publish(marker);

  // Publish orientation error based on axis-angle
  marker.header.frame_id = this->ros_params_.robot_base_link;
  marker.header.stamp = get_node()->now();
  marker.id = 1;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.points.resize(2);
  marker.points[0].x = 0.0;
  marker.points[0].y = 0.0;
  marker.points[0].z = 0.0;
  marker.points[1].x = error_angle_axis.axis()(0) * error_angle_axis.angle();
  marker.points[1].y = error_angle_axis.axis()(1) * error_angle_axis.angle();
  marker.points[1].z = error_angle_axis.axis()(2) * error_angle_axis.angle();
  //RCLCPP_INFO(get_node()->get_logger(), "Error: %f, %f, %f", error_angle_axis.axis()(0) * error_angle_axis.angle(), error_angle_axis.axis()(1) * error_angle_axis.angle(), error_angle_axis.axis()(2) * error_angle_axis.angle());
  marker.scale.x = 0.05;
  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  this->marker_publisher_->publish(marker);
}

bool TaskSpaceTrajectoryController::validate_trajectory_msg(const moveit_msgs::msg::CartesianTrajectory & msg)
{
  if(msg.points.size() == 0){
    RCLCPP_ERROR(get_node()->get_logger(), "Received empty trajectory");
    return false;
  }
  if(msg.header.frame_id != this->ros_params_.robot_base_link){
    RCLCPP_ERROR(get_node()->get_logger(), "Received trajectory with base frame %s. Expected %s", msg.header.frame_id.c_str(), this->ros_params_.robot_base_link.c_str());
    return false;
  }
  if(msg.tracked_frame != this->ros_params_.end_effector_link){
    RCLCPP_ERROR(get_node()->get_logger(), "Received trajectory with tracked frame %s. Expected %s", msg.tracked_frame.c_str(), this->ros_params_.robot_base_link.c_str());
    return false;
  }

  const auto & first_point = msg.points[0];
  rclcpp::Duration first_time_from_start = first_point.time_from_start;
  const auto actual_point = get_holding_position();
  const auto euclidean_distance = [](const moveit_msgs::msg::CartesianPoint & a, const moveit_msgs::msg::CartesianPoint & b){
    return std::sqrt(std::pow(a.pose.position.x - b.pose.position.x, 2) + std::pow(a.pose.position.y - b.pose.position.y, 2) + std::pow(a.pose.position.z - b.pose.position.z, 2));
  };

  // Check if the first point is too far from the actual position
  if(first_time_from_start == rclcpp::Duration(0,0)){
    const double distance = euclidean_distance(first_point.point, actual_point->point);
    if(distance > this->ros_params_.action_server.first_point_tolerance){
      RCLCPP_ERROR(get_node()->get_logger(), "Received trajectory with first point too far from actual position. Distance: %f. Tolerance: %f", distance, this->ros_params_.action_server.first_point_tolerance);
      return false;
    }
  }

  // Check if trajectory points are correctly ordered
  for(auto point_itr = msg.points.begin(); point_itr != msg.points.end(); point_itr++){
    const auto & point = *point_itr;
    if(point_itr != msg.points.begin()){
      const auto & previous_point = *(point_itr - 1);
      rclcpp::Duration point_time_from_start = point.time_from_start;
      rclcpp::Duration previous_point_time_from_start = previous_point.time_from_start;
      if(point_time_from_start <= previous_point_time_from_start){
        RCLCPP_ERROR(get_node()->get_logger(), "Received trajectory with points not correctly ordered");
        return false;
      }
    }
  }
  // for(const auto & point : msg.points){
  //   point.time_from_start
  // }
  return true;
}

inline TaskSpaceTrajectoryStatus TaskSpaceTrajectoryController::get_controller_status(){
  return *(this->rt_controller_status_.readFromRT());
}

std::shared_ptr<moveit_msgs::msg::CartesianTrajectoryPoint> TaskSpaceTrajectoryController::get_holding_position(){
  // Initialize reference
  std::shared_ptr<moveit_msgs::msg::CartesianTrajectoryPoint> reference_msg = std::make_shared<moveit_msgs::msg::CartesianTrajectoryPoint>();
  reference_msg->point.pose.position.x = this->actual_ee_position_(0);
  reference_msg->point.pose.position.y = this->actual_ee_position_(1);
  reference_msg->point.pose.position.z = this->actual_ee_position_(2);
  reference_msg->point.pose.orientation.x = this->actual_ee_orientation_.x();
  reference_msg->point.pose.orientation.y = this->actual_ee_orientation_.y();
  reference_msg->point.pose.orientation.z = this->actual_ee_orientation_.z();
  reference_msg->point.pose.orientation.w = this->actual_ee_orientation_.w();
  // No velocity
  reference_msg->point.velocity.linear.x = 0.0;
  reference_msg->point.velocity.linear.y = 0.0;
  reference_msg->point.velocity.linear.z = 0.0;
  reference_msg->point.velocity.angular.x = 0.0;
  reference_msg->point.velocity.angular.y = 0.0;
  reference_msg->point.velocity.angular.z = 0.0;
  return reference_msg;
}

// Callbacks
void TaskSpaceTrajectoryController::command_callback(const std::shared_ptr<moveit_msgs::msg::CartesianTrajectoryPoint> msg){
  if(!this->is_active){
    RCLCPP_ERROR(get_node()->get_logger(), "Received command but controller is not active");
    return;
  }
  if(get_controller_status() != TaskSpaceTrajectoryStatus::IDLE){
    RCLCPP_ERROR(get_node()->get_logger(), "Controller is busy. Rejecting command");
    return;
  }
  // Set command
  this->command_buffer_.writeFromNonRT(msg);
}

rclcpp_action::GoalResponse TaskSpaceTrajectoryController::goal_received_callback(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const FollowCartesianTrajectoryAction::Goal> goal){
  RCLCPP_INFO(get_node()->get_logger(), "Received new action goal");

  if(!this->is_active){
    RCLCPP_ERROR(get_node()->get_logger(), "Rejecting action goal because controller is not active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if(!validate_trajectory_msg(goal->trajectory)){
    RCLCPP_ERROR(get_node()->get_logger(), "Rejecting invalid trajectory");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if(get_controller_status() != TaskSpaceTrajectoryStatus::IDLE){
    RCLCPP_ERROR(get_node()->get_logger(), "Controller is busy. Rejecting action goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TaskSpaceTrajectoryController::goal_cancelled_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowCartesianTrajectoryAction>> goal_handle){
  RCLCPP_INFO(get_node()->get_logger(), "Received request to cancel goal");

  const RealtimeGoalHandlePtr active_goal = *(this->rt_active_goal_.readFromRT());
  if (active_goal && active_goal->gh_ == goal_handle){
    RCLCPP_INFO(get_node()->get_logger(), "Active action goal canceled successfully.");

    // Cancel the currently active goal
    this->rt_controller_status_.writeFromNonRT(TaskSpaceTrajectoryStatus::IDLE);
    std::shared_ptr<FollowCartesianTrajectoryAction::Result> action_res = std::make_shared<FollowCartesianTrajectoryAction::Result>();
    action_res->error_code = action_res->REQUESTED_CANCEL;
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

    // Set holding position as reference
    const auto reference_msg = get_holding_position();
    this->command_buffer_.writeFromNonRT(reference_msg);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TaskSpaceTrajectoryController::goal_accepted_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowCartesianTrajectoryAction>> goal_handle){
  // Update new trajectory
  {
    auto traj_msg =std::make_shared<moveit_msgs::msg::CartesianTrajectory>(goal_handle->get_goal()->trajectory);
    this->current_trajectory_.writeFromNonRT(traj_msg);
  }

  // Create a real-time goal handle with the goal received from the action server
  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  // Signal the start of the goal handling
  rt_goal->execute();
  // Set the active goal
  this->rt_active_goal_.writeFromNonRT(rt_goal);

  // Mark the controller status as tracking trajectory
  this->rt_controller_status_.writeFromNonRT(TaskSpaceTrajectoryStatus::TRACKING_TRAJECTORY);

  // Delete the old goal handle timer
  this->goal_handle_timer_.reset();

  // Setup goal status checking timer
  this->goal_handle_timer_ = get_node()->create_wall_timer(
    this->action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}

}

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(task_space_trajectory_controller::TaskSpaceTrajectoryController,
                       controller_interface::ControllerInterface)
