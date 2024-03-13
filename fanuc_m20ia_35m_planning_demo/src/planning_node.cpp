#include "planning_node.hpp"
#include "planning_node_parameters.hpp"

namespace planning_node
{

  PlanningNode::PlanningNode(const std::string node_name)
      : Node(node_name)
  {
    this->param_listener_ = std::make_shared<planning_node::ParamListener>(this->get_node_parameters_interface());
    RCLCPP_INFO(this->get_logger(), "input_bag_file_topic value: %s", this->get_parameter("input_bag_file_topic").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "planning_group value: %s", this->get_parameter("planning_group").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "recorded_trajectory_home_pose_topic value: %s", this->get_parameter("recorded_trajectory_home_pose_topic").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "recorded_trajectory_point2point_topic value: %s", this->get_parameter("recorded_trajectory_point2point_topic").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "recorded_trajectory_cartesian_topic value: %s", this->get_parameter("recorded_trajectory_cartesian_topic").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "feedback_sampling_period value: %ld", this->get_parameter("feedback_sampling_period").as_int());
    RCLCPP_INFO(this->get_logger(), "max_velocity_scaling_factor value: %f", this->get_parameter("max_velocity_scaling_factor").as_double());
    RCLCPP_INFO(this->get_logger(), "recorded_trajectory_file_name value: %s", this->get_parameter("recorded_trajectory_file_name").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "goal_position_tolerance value: %f", this->get_parameter("goal_position_tolerance").as_double());
    RCLCPP_INFO(this->get_logger(), "goal_orientation_tolerance value: %f", this->get_parameter("goal_orientation_tolerance").as_double());
    RCLCPP_INFO(this->get_logger(), "goal_joint_tolerance value: %f", this->get_parameter("goal_joint_tolerance").as_double());
    RCLCPP_INFO(this->get_logger(), "controller_name value: %s", this->get_parameter("controller_name").as_string().c_str());
    RCLCPP_INFO(this->get_logger(), "controller_type value: %s", this->get_parameter("controller_type").as_string().c_str());

    if (this->get_parameter("controller_type").as_string().compare("TaskSpaceTrajectoryController") == 0)
    {
      this->action_name_ = "/" + this->get_parameter("controller_name").as_string() + "/follow_cartesian_trajectory";
      this->action_type_ = "cartesian_control_msgs/action/FollowCartesianTrajectory_FeedbackMessage";

      this->task_space_client_ptr_ = rclcpp_action::create_client<SendTaskSpaceTrajectory>(
          this,
          this->action_name_.c_str());
    }
    else if (this->get_parameter("controller_type").as_string().compare("JointTrajectoryController") == 0)
    {

      this->action_name_ = "/" + this->get_parameter("controller_name").as_string() + "/follow_joint_trajectory";
      this->action_type_ = "control_msgs/action/FollowJointTrajectory_FeedbackMessage";

      this->joint_space_client_ptr_ = rclcpp_action::create_client<SendJointSpaceTrajectory>(
          this,
          this->action_name_.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Controller type not recognized. Please check the controller_type parameter");
      rclcpp::shutdown();
      exit(EXIT_FAILURE);
    }

    this->trajectory_ = trajectory_;

    this->set_recorded_trajectory_file_name(this->get_parameter("recorded_trajectory_file_name").as_string().c_str());

    this->recorded_trajectory_home_pose_topic_ = this->get_parameter("recorded_trajectory_home_pose_topic").as_string().c_str();
    this->recorded_trajectory_point2point_topic_ = this->get_parameter("recorded_trajectory_point2point_topic").as_string().c_str();
    this->recorded_trajectory_cartesian_topic_ = this->get_parameter("recorded_trajectory_cartesian_topic").as_string().c_str();

    this->writer_ = std::make_unique<rosbag2_cpp::Writer>();
    rosbag2_storage::StorageOptions storage_options = rosbag2_storage::StorageOptions();
    storage_options.uri = recorded_trajectory_file_name_;
    storage_options.max_bagfile_size = 0;
    this->writer_->open(storage_options);

    rosbag2_storage::TopicMetadata topic_metadata = rosbag2_storage::TopicMetadata();
    topic_metadata.name = this->recorded_trajectory_home_pose_topic_;
    topic_metadata.type = this->action_type_;
    topic_metadata.serialization_format = "cdr";
    topic_metadata.offered_qos_profiles = "";
    this->writer_->create_topic(topic_metadata);
    topic_metadata.name = this->recorded_trajectory_point2point_topic_;
    this->writer_->create_topic(topic_metadata);
    topic_metadata.name = this->recorded_trajectory_cartesian_topic_;
    this->writer_->create_topic(topic_metadata);

    this->feedback_sampling_period_ = this->get_parameter("feedback_sampling_period").as_int();
    this->feedback_count_ = 0;

    this->declare_parameter<std::string>("writer_topic", this->recorded_trajectory_home_pose_topic_);
    RCLCPP_INFO(this->get_logger(), "writer_topic value: %s", this->get_parameter("writer_topic").as_string().c_str());
  }

  std::shared_future<rclcpp_action::ClientGoalHandle<PlanningNode::SendTaskSpaceTrajectory>::WrappedResult> PlanningNode::task_space_send_goal(const moveit_msgs::msg::CartesianTrajectory trajectory)
  {
    using namespace std::placeholders;
    this->feedback_count_ = 0;

    if (this->get_parameter("controller_type").as_string().compare("TaskSpaceTrajectoryController") != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Task Space Goal was rejected. Controller type is not TaskSpaceTrajectoryController");
      exit(EXIT_FAILURE);
    }

    if (!this->task_space_client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "Task Space action server not available after waiting. Check if the right controller is running and the action server is up!");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }

    auto goal_msg = SendTaskSpaceTrajectory::Goal();
    goal_msg.set__trajectory(trajectory);

    RCLCPP_INFO(this->get_logger(), "Sending task space trajectory to the action server");

    auto send_goal_options = rclcpp_action::Client<SendTaskSpaceTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&PlanningNode::task_space_goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&PlanningNode::task_space_feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&PlanningNode::task_space_result_callback, this, _1);

    std::shared_future<std::shared_ptr<PlanningNode::GoalHandleSendTaskSpaceTrajectory>> send_goal_future = this->task_space_client_ptr_->async_send_goal(goal_msg, send_goal_options);

    if (!send_goal_future.valid())
    {
      RCLCPP_INFO(this->get_logger(), "REJECTED TASK SPACE SENDING GOAL");
      rclcpp::shutdown();
      exit(EXIT_FAILURE);
    }

    send_goal_future.wait();

    return this->task_space_client_ptr_->async_get_result(send_goal_future.get());
  }

  void PlanningNode::task_space_goal_response_callback(const GoalHandleSendTaskSpaceTrajectory::SharedPtr &goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Task Space Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Task space trajectory received. Goal accepted by server, waiting for result");
    }
  }

  void PlanningNode::task_space_feedback_callback(GoalHandleSendTaskSpaceTrajectory::SharedPtr, const std::shared_ptr<const cartesian_control_msgs::action::FollowCartesianTrajectory::Feedback> feedback)
  {
    if (this->feedback_count_ % this->feedback_sampling_period_ == 0)
    {
      std::string writer_topic = this->get_parameter("writer_topic").as_string().c_str();
      cartesian_control_msgs::action::FollowCartesianTrajectory_FeedbackMessage::SharedPtr msg = std::make_shared<cartesian_control_msgs::action::FollowCartesianTrajectory_FeedbackMessage>();
      std::shared_ptr<rclcpp::SerializedMessage> serialized_feedback = std::make_shared<rclcpp::SerializedMessage>();
      msg->feedback = *feedback;
      rclcpp::Serialization<cartesian_control_msgs::action::FollowCartesianTrajectory_FeedbackMessage> serialization_;
      serialization_.serialize_message(msg.get(), serialized_feedback.get());
      this->writer_->write(serialized_feedback, writer_topic.c_str(), this->action_type_, this->now());
      //RCLCPP_INFO(this->get_logger(), "WRITTEN MESSAGE ON %s", this->get_parameter("writer_topic").as_string().c_str());
    }

    this->feedback_count_++;
  }

  void PlanningNode::task_space_result_callback(const GoalHandleSendTaskSpaceTrajectory::WrappedResult &result)
  {
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Task Space Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Task Space Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Task Space Unknown result code");
      return;
    }

    std::stringstream ss;
    ss << "Task Space Goal Result:  SUCCESS";

    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  /******************************************************************************************************************************
   *
   * JOINT SPACE METHOD
   *
   * ***************************************************************************************************************************/
  std::shared_future<rclcpp_action::ClientGoalHandle<PlanningNode::SendJointSpaceTrajectory>::WrappedResult> PlanningNode::joint_space_send_goal(const trajectory_msgs::msg::JointTrajectory trajectory)
  {
    using namespace std::placeholders;

    if (this->get_parameter("controller_type").as_string().compare("JointTrajectoryController") != 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Joint Space Goal was rejected. Controller type is not JointTrajectoryController");
      rclcpp::shutdown();
      exit(EXIT_FAILURE);
    }

    if (!this->joint_space_client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "Joint Space action server not available after waiting. Check if the right controller is running and the action server is up!");
        rclcpp::shutdown();
        exit(EXIT_FAILURE);
    }

    auto goal_msg = SendJointSpaceTrajectory::Goal();
    goal_msg.set__trajectory(trajectory);

    RCLCPP_INFO(this->get_logger(), "Sending joint space trajectory to the action server");

    auto send_goal_options = rclcpp_action::Client<SendJointSpaceTrajectory>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&PlanningNode::joint_space_goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&PlanningNode::joint_space_feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&PlanningNode::joint_space_result_callback, this, _1);

    std::shared_future<std::shared_ptr<PlanningNode::GoalHandleSendJointSpaceTrajectory>> send_goal_future = this->joint_space_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    send_goal_future.wait();

    return this->joint_space_client_ptr_->async_get_result(send_goal_future.get());
  }

  void PlanningNode::joint_space_goal_response_callback(const GoalHandleSendJointSpaceTrajectory::SharedPtr &goal_handle)
  {
    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Joint Space Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Joint space trajectory received. Goal accepted by server, waiting for result");
    }
  }

  void PlanningNode::joint_space_feedback_callback(GoalHandleSendJointSpaceTrajectory::SharedPtr, const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> feedback)
  {
    if (this->feedback_count_ % feedback_sampling_period_ == 0)
    {
      std::string writer_topic = this->get_parameter("writer_topic").as_string().c_str();
      control_msgs::action::FollowJointTrajectory_FeedbackMessage::SharedPtr msg = std::make_shared<control_msgs::action::FollowJointTrajectory_FeedbackMessage>();
      std::shared_ptr<rclcpp::SerializedMessage> serialized_feedback = std::make_shared<rclcpp::SerializedMessage>();
      msg->feedback = *feedback;
      rclcpp::Serialization<control_msgs::action::FollowJointTrajectory_FeedbackMessage> serialization_;
      serialization_.serialize_message(msg.get(), serialized_feedback.get());
      this->writer_->write(serialized_feedback, writer_topic.c_str(), this->action_type_, this->now());

      //RCLCPP_INFO(this->get_logger(), "WRITTEN MESSAGE ON %s", writer_topic.c_str());
    }
    this->feedback_count_++;
  }

  void PlanningNode::joint_space_result_callback(const GoalHandleSendJointSpaceTrajectory::WrappedResult &result)
  {
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Joint Space Goal was aborted");
      rclcpp::shutdown();
      exit(EXIT_FAILURE);
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Joint Space Goal was canceled");
      rclcpp::shutdown();
      exit(EXIT_FAILURE);
    default:
      RCLCPP_ERROR(this->get_logger(), "Joint Space Unknown result code");
      rclcpp::shutdown();
      exit(EXIT_FAILURE);
    }

    std::stringstream ss;
    ss << "Joint Space Goal Result: SUCCESS";

    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void PlanningNode::set_recorded_trajectory_file_name(std::string recorded_trajectory_file_name)
  {
    std::time_t t = std::time(nullptr);
    std::tm *now = std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(now, "%Y-%m-%d_%H-%M-%S");

    std::string date_string = oss.str();
    std::string underscore = "_";
    this->recorded_trajectory_file_name_ = recorded_trajectory_file_name + underscore + date_string;
  }

  moveit_msgs::msg::CartesianTrajectory PlanningNode::get_task_space_trajectory(moveit_msgs::msg::RobotTrajectory trajectory, moveit::core::RobotState &robot_state, const rclcpp::Logger *logger, const std::string target_frame, std::string frame_id, std::string planning_group)
  {
    long unsigned int j = 0;

    auto joint_names = trajectory.joint_trajectory.joint_names;

    auto num_points = trajectory.joint_trajectory.points.size();

    geometry_msgs::msg::Twist task_space_velocities = geometry_msgs::msg::Twist();

    moveit_msgs::msg::CartesianTrajectory cartesian_trajectory_msg = moveit_msgs::msg::CartesianTrajectory();

    moveit_msgs::msg::CartesianTrajectoryPoint planned_cartesian_point = moveit_msgs::msg::CartesianTrajectoryPoint();

    std::vector<moveit_msgs::msg::CartesianTrajectoryPoint> cartesian_trajectory_points = std::vector<moveit_msgs::msg::CartesianTrajectoryPoint>();

    geometry_msgs::msg::Pose planned_eef_pose = geometry_msgs::msg::Pose();

    for (j = 0; j < num_points; j++)
    {
      planned_eef_pose = direct_kinematic(robot_state, target_frame, joint_names, trajectory.joint_trajectory.points[j].positions);
      task_space_velocities = first_order_kinematic(robot_state, joint_names, trajectory.joint_trajectory.points[j].positions, trajectory.joint_trajectory.points[j].velocities, planning_group);
      // CARTESIAN TRAJECTORY POINT
      planned_cartesian_point.point.set__pose(planned_eef_pose);
      planned_cartesian_point.set__time_from_start(trajectory.joint_trajectory.points[j].time_from_start);
      planned_cartesian_point.point.set__velocity(task_space_velocities);

      // POPULATE CARTESIAN TRAJECTORY POINTS
      cartesian_trajectory_points.push_back(planned_cartesian_point);
    }
    // BUILD CARTESIAN TRAJECTORY MSG

    // HEADER
    std_msgs::msg::Header header = std_msgs::msg::Header();
    builtin_interfaces::msg::Time stamp = builtin_interfaces::msg::Time();

    stamp.set__sec(0);
    stamp.set__nanosec(0);
    header.set__stamp(stamp);
    header.set__frame_id(frame_id);

    cartesian_trajectory_msg.set__header(header);

    // CARTESIAN TRAJECTORY POINT[]
    cartesian_trajectory_msg.set__points(cartesian_trajectory_points);

    // TRACKED FRAME
    cartesian_trajectory_msg.set__tracked_frame(target_frame);

    RCLCPP_INFO(*logger, "Extracted task space trajectory. Trajectory points number: %ld", cartesian_trajectory_msg.points.size());

    return cartesian_trajectory_msg;
  }

  geometry_msgs::msg::Pose PlanningNode::direct_kinematic(moveit::core::RobotState &robot_state, const std::string target_frame, std::vector<std::string> joint_names, trajectory_msgs::msg::JointTrajectoryPoint::_positions_type positions)
  {
    long unsigned int i = 0;

    auto num_joints = joint_names.size();

    geometry_msgs::msg::Pose planned_eef_pose = geometry_msgs::msg::Pose();
    Eigen::Isometry3d isometry = Eigen::Isometry3d();
    geometry_msgs::msg::TransformStamped t = geometry_msgs::msg::TransformStamped();

    for (i = 0; i < num_joints; i++)
    {
      robot_state.setJointPositions(joint_names[i], &positions[i]);
      // robot_state.setJointVelocities(robot_state.getJointModel(joint_names[i]), &trajectory.joint_trajectory.points[j].velocities[i]);
    }

    robot_state.update();

    isometry = robot_state.getGlobalLinkTransform(target_frame);

    t = tf2::eigenToTransform(isometry);

    planned_eef_pose.position.set__x(t.transform.translation.x);
    planned_eef_pose.position.set__y(t.transform.translation.y);
    planned_eef_pose.position.set__z(t.transform.translation.z);

    planned_eef_pose.orientation.set__x(t.transform.rotation.x);
    planned_eef_pose.orientation.set__y(t.transform.rotation.y);
    planned_eef_pose.orientation.set__z(t.transform.rotation.z);
    planned_eef_pose.orientation.set__w(t.transform.rotation.w);

    return planned_eef_pose;
  }

  geometry_msgs::msg::Twist PlanningNode::first_order_kinematic(moveit::core::RobotState &robot_state, std::vector<std::string> joint_names, trajectory_msgs::msg::JointTrajectoryPoint::_positions_type positions, trajectory_msgs::msg::JointTrajectoryPoint::_velocities_type velocities, std::string planning_group)
  {
    long unsigned int i = 0;
    auto num_joints = joint_names.size();
    int task_spaceDoF = 6;

    Eigen::MatrixXd jacobian = Eigen::MatrixXd(task_spaceDoF, joint_names.size());
    Eigen::Matrix<double, Eigen::Dynamic, 1> joints_velocities(joint_names.size(), 1);
    geometry_msgs::msg::Twist task_space_velocities = geometry_msgs::msg::Twist();
    geometry_msgs::msg::Vector3 linear = geometry_msgs::msg::Vector3();
    geometry_msgs::msg::Vector3 angular = geometry_msgs::msg::Vector3();

    for (i = 0; i < num_joints; i++)
    {
      robot_state.setJointPositions(joint_names[i], &positions[i]);
      joints_velocities(i, 0) = velocities[i];
    }

    robot_state.update();

    // FIRST ORDER KINEMATIC
    jacobian = robot_state.getJacobian(robot_state.getJointModelGroup(planning_group));

    auto v = jacobian * joints_velocities;

    linear.x = v(0, 0);
    linear.y = v(1, 0);
    linear.z = v(2, 0);

    angular.x = v(3, 0);
    angular.y = v(4, 0);
    angular.z = v(5, 0);

    // TWIST
    task_space_velocities.set__angular(angular);
    task_space_velocities.set__linear(linear);

    return task_space_velocities;
  }

  std::pair<double, moveit::planning_interface::MoveGroupInterface::Plan> PlanningNode::cartesian_path_planning(std::vector<geometry_msgs::msg::Pose> waypoints, const rclcpp::Logger *logger, moveit::planning_interface::MoveGroupInterface *move_group_interface, moveit_msgs::msg::Constraints &constraints)
  {

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, constraints);

    RCLCPP_INFO(*logger, "Achieved %.2f%% of Cartesian path", fraction * 100.0); /**< Tracking the result of the cartesian path planning.*/
    robotStateToRobotStateMsg(*move_group_interface->getCurrentState(), plan.start_state_);

    plan.trajectory_ = trajectory;
    return std::make_pair(fraction, plan);
  }

  bool PlanningNode::plan_validation(moveit::planning_interface::MoveGroupInterface::Plan plan, geometry_msgs::msg::Pose desired_position, moveit::core::RobotState &robot_state, double tolerance, const rclcpp::Logger *logger, std::string target_frame)
  {
    auto last_point_index = plan.trajectory_.joint_trajectory.points.size() - 1;
    auto last_planned_point = plan.trajectory_.joint_trajectory.points[last_point_index];
    auto joint_names = plan.trajectory_.joint_trajectory.joint_names;
    auto last_planned_positions = last_planned_point.positions;

    for (long unsigned int i = 0; i < joint_names.size(); i++)
    {
      robot_state.setJointPositions(joint_names[i], &last_planned_positions[i]);
    }

    robot_state.update();

    // auto last_joint = joint_names[joint_names.size()-1];
    // robot_state.getJointTransform(last_joint);

    auto isometry = robot_state.getGlobalLinkTransform(target_frame);

    geometry_msgs::msg::TransformStamped t;
    t = tf2::eigenToTransform(isometry);

    auto planned_position = t.transform.translation;

    double error = (std::pow((desired_position.position.x - planned_position.x), 2) + std::pow((desired_position.position.y - planned_position.y), 2) + std::pow((desired_position.position.z - planned_position.z), 2));

    RCLCPP_INFO(*logger, "Value Error between desired and planned position: %f, %f", error, std::pow(tolerance, 2));
    RCLCPP_INFO(*logger, "Value Error between desired and planned position is less than tolerance: %d", error < std::pow(tolerance, 2));

    return error < std::pow(tolerance, 2);
  }

} // namespace planning_node