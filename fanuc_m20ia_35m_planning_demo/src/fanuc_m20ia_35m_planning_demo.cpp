#include "fanuc_m20ia_35m_planning_demo.hpp"


std::shared_ptr<geometry_msgs::msg::PoseArray> read_cartesian_path_poses(const rclcpp::Logger &logger, std::string bag_filename, std::string topic_name){
  rosbag2_cpp::Reader reader_;  /**< Reader object for opening and reading bag files */

  rclcpp::Serialization<geometry_msgs::msg::PoseArray> serialization_;

  const geometry_msgs::msg::PoseArray::SharedPtr ros_msg = std::make_shared<geometry_msgs::msg::PoseArray>(); /**<ROS 2 deserialized message which will hold the result of our deserialization that is a PoseArray.*/

  reader_.open(bag_filename);

  while (reader_.has_next()) { /**< loop through messages in the bag until we read a message recorded from our desired topic.*/

      rosbag2_storage::SerializedBagMessageSharedPtr msg = reader_.read_next(); 

      if (msg->topic_name.compare(topic_name) == 0) {
        continue;
      }

      rclcpp::SerializedMessage serialized_msg(*msg->serialized_data); /**<construct an rclcpp::SerializedMessage object from the serialized data we just read*/

      serialization_.deserialize_message(&serialized_msg, ros_msg.get()); /**<pass both rclcpp::SerializedMessage and ROS 2 deserialized message objects to the rclcpp::Serialization::deserialize_message method.*/
      RCLCPP_INFO(logger, "Got the path from the bag file \n");
  }
  return ros_msg; /**<PoseArray read from the bag file whose value is returned to cartesian_path_poses variable*/
}

/*******************************************************************************
  * CLOSURES FOR VISUALIZATION
  *
  * The following closures are requested so as to help render visualizations in RViz.
  * Each of the four closures capture moveit_visual_tools by reference. 
  * 
  * The first one, draw_title, adds text one meter and half above the base of the robot. This is a useful way to show the state of your program from a high level.
  * 
  * The second draws the desired path given in input by the user
  * 
  * The third draws the tool path of a trajectory that we have planned. This is often helpful for understanding a planned trajectory from the perspective of the tool.
  * 
  * The last one draws the start point and the target point of a point-to-point planning. It is useful for checking if the points are correct and to visualize them 
  * regardless of whether the planning is successful or not.
  ******************************************************************************/

void draw_title(moveit_visual_tools::MoveItVisualTools &moveit_visual_tools, std::string text){
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.50;  // Place text 1.5m above the base link
  moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XXXLARGE);
  moveit_visual_tools.trigger();
}

void draw_cartesian_path(moveit_visual_tools::MoveItVisualTools &moveit_visual_tools, const std::shared_ptr<geometry_msgs::msg::PoseArray> path_poses){
  moveit_visual_tools.publishPath(path_poses->poses);
  moveit_visual_tools.trigger();
}

void draw_planned_trajectory(moveit_visual_tools::MoveItVisualTools &moveit_visual_tools, moveit::planning_interface::MoveGroupInterface &move_group_interface, const std::string &planning_group, const moveit_msgs::msg::RobotTrajectory &trajectory){
  const moveit::core::RobotModelConstPtr robot_model = move_group_interface.getRobotModel();
  const moveit::core::JointModelGroup *jmg = robot_model->getJointModelGroup(planning_group);
  const moveit::core::LinkModel *link = robot_model->getLinkModel(move_group_interface.getEndEffectorLink());
  moveit_visual_tools.publishTrajectoryLine(trajectory, link, jmg);
  moveit_visual_tools.trigger();
}

void draw_start_and_target_points(moveit_visual_tools::MoveItVisualTools &moveit_visual_tools, const geometry_msgs::msg::PoseStamped &start_pose, const geometry_msgs::msg::PoseStamped &target_pose){
  geometry_msgs::msg::Vector3 scale;
  scale.x = 0.05;
  scale.y = 0.05;
  scale.z = 0.05;
  moveit_visual_tools.publishSphere(start_pose, rviz_visual_tools::Colors::RED, scale);
  moveit_visual_tools.publishSphere(target_pose, rviz_visual_tools::Colors::GREEN, scale);
  moveit_visual_tools.trigger(); 
}

/******************************************************************************
 * CLOSURE FOR  EXECUTION
 * 
 * If the planning has been successful the trajectory will be executed otherwise is notified that the execution has failed.
 * 
******************************************************************************/

/*! 
  * \brief Execute the planned trajectory and notify the user if the execution has failed.
  * @param moveit_visual_tools: the MoveItVisualTools object
  * @param trajectory: the trajectory to be executed
  * @param node: the PlanningNode object
  * 
*/
template <typename TrajectoryType>
void execute_trajectory(moveit_visual_tools::MoveItVisualTools &moveit_visual_tools, const TrajectoryType &trajectory, std::shared_ptr<planning_node::PlanningNode> node){
  moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to execute the planned trajectory"); /**<Wait for the Next button to be pressed to execute*/
  draw_title(moveit_visual_tools, "Executing"); /**<Draw the text passed as parameter in order to notify that execution is in progress.*/
  moveit_visual_tools.trigger(); /**<this method rendered in RViz the text drawn with "draw_title". The reason for this is that messages sent to RViz are batched up and sent when you call trigger to reduce bandwidth of the marker topics.*/
  auto future = node->execute_trajectory(trajectory);
  future.wait();
}

// auto const remove_collision_objects = [&planning_scene_interface]()
// {
//   std::vector<std::string> keys;
//   for(auto entry : planning_scene_interface.getObjects())
//     keys.push_back(entry.first);
//   planning_scene_interface.removeCollisionObjects(keys);
// };

/***********************************************************************************************
 * Function FOR READING A PATH FROM A BAG FILE 
 *
 * The function instantiate a rosbag2_cpp::Reader object.
 * 
 * Then it constructs an rclcpp::SerializedMessage object from the serialized data we just read.
 *  
 * Additionally, we need to create a ROS 2 deserialized message which will hold the result 
 * of the deserialization.
 * 
 * Then, both these objects are passed to the rclcpp::Serialization::deserialize_message method
 * in order to obtain the deserialized PoseArray.
 ************************************************************************************************/

const std::vector<geometry_msgs::msg::Pose> populate_waypoints(const std::shared_ptr<geometry_msgs::msg::PoseArray> cartesian_path_poses){
  std::vector<geometry_msgs::msg::Pose> waypoints;
    
  for(long unsigned int i = 0; i < cartesian_path_poses->poses.size(); i++){
    waypoints.push_back(cartesian_path_poses->poses[i]); /**< Populating wayoints with the point of the path read from the bag file in input*/
  }  

  return waypoints;
}


int main(int argc, char * argv[])
{   
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Create planning node
  std::shared_ptr<planning_node::PlanningNode> node = std::make_shared<planning_node::PlanningNode>(
    "planning_node"
  );

  // Get node logger
  auto const logger = node->get_logger();

  // Spin the node
  /* 
  Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS.
  Before being able to initialize MoveItVisualTools, it needs to have a executor spinning on our ROS node. 
  This is necessary because of how MoveItVisualTools interacts with ROS services and topics.
  We have to add our executor before creating the MoveIt MoveGroup Interface.
  */
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]()
  { 
    executor.spin();
    rclcpp::shutdown();
  });

  // Get the bag file path from the command line
  if (argc < 2) {
      RCLCPP_ERROR(logger, "Insufficient number of arguments. Requested 2 arguments but %d were given", argc);
      return 1;
  }
  const std::filesystem::path relative_path_bag_filename = std::filesystem::path(argv[1]);
  RCLCPP_INFO(logger, "Argument: %s", relative_path_bag_filename.c_str());
  const std::string bag_filename = std::filesystem::absolute(relative_path_bag_filename);
  RCLCPP_INFO(logger, "Reading path from bagfile: %s", bag_filename.c_str());

  // Check if we are using a task space controller or a joint space controller
  // Notice that the parameter value is validated by generate_parameter_library
  const std::string controller_type = node->get_parameter("controller_type").as_string();

  // Flag for reference generation: when it is true, references are generated in the task space; otherwise, they are generated in the joint space.
  const bool is_task_space_controller = controller_type.compare("TaskSpaceTrajectoryController")==0; 

  // Read the path from the bag file
  std::string input_bag_file_topic = node->get_parameter("input_bag_file_topic").value_to_string();

  const std::shared_ptr<geometry_msgs::msg::PoseArray> cartesian_path_poses = read_cartesian_path_poses(logger, bag_filename, input_bag_file_topic);
  
  // Check that in the file there is at least one pose otherwise notify the error.
  if(cartesian_path_poses->poses.size() < 1){
    RCLCPP_ERROR(logger, "The provided bagfile is empty. Aborting...");
    rclcpp::shutdown();
    return 1;
  }

  // Get base frame's name from node's parameter
  std::string base_frame = node->get_parameter("base_frame").as_string();

  auto cartesian_path_first_pose = geometry_msgs::msg::PoseStamped();
  cartesian_path_first_pose.set__pose(cartesian_path_poses->poses[0]); 

  std_msgs::msg::Header header = std_msgs::msg::Header();
  header.stamp.sec = 0.0;

  header.frame_id = base_frame;
  cartesian_path_first_pose.set__header(header); 

  // Get the name of the bag file where the executed trajectory is going to be saved from node's parameter
  std::string recorded_trajectory_file_name = node->get_parameter("recorded_trajectory_file_name").as_string().c_str();

  // Create the MoveIt MoveGroup Interface and configure it
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const std::string planning_group = node->get_parameter("planning_group").as_string().c_str();
  moveit::planning_interface::MoveGroupInterface move_group_interface {node, planning_group};
  move_group_interface.setGoalPositionTolerance(node->get_parameter("goal_position_tolerance").as_double());
  move_group_interface.setGoalOrientationTolerance(node->get_parameter("goal_orientation_tolerance").as_double());
  move_group_interface.setGoalJointTolerance(node->get_parameter("goal_joint_tolerance").as_double());
  move_group_interface.setPlanningTime(45.0);
  move_group_interface.setMaxVelocityScalingFactor(node->get_parameter("max_velocity_scaling_factor").as_double());
  move_group_interface.startStateMonitor();
  
  // Create a MoveItVisualTools object
  moveit_visual_tools::MoveItVisualTools moveit_visual_tools {node,
    base_frame,
    rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface.getRobotModel()
  };
  moveit_visual_tools.loadRemoteControl();

  const std::string end_effector_link_name = move_group_interface.getEndEffectorLink();
  RCLCPP_INFO(logger, "End effector name: %s", end_effector_link_name.c_str());
  auto robot_state = moveit::core::RobotState(move_group_interface.getRobotModel()); 

  moveit::core::MoveItErrorCode success;

  /*******************************************************************************
   * 1. MOVE TO HOME CONFIGURATION
   ******************************************************************************/
  auto named_targets = move_group_interface.getNamedTargets();
  std::string home_target = node->get_parameter("named_targets.home_pose").as_string();
  std::string home_inverse_target = node->get_parameter("named_targets.home_pose_inverse").as_string();

  // If in the SRDF two home positions are defined representing two convenient robot configurations (possibly mirror images of each other), then planning is performed towards one of these named targets; otherwise, this step is skipped.
  if( std::find(named_targets.begin(),named_targets.end(), home_target) != named_targets.end() && std::find(named_targets.begin(),named_targets.end(), home_inverse_target) != named_targets.end()){
    node->set_parameter(rclcpp::Parameter("writer_topic", node->get_parameter("recorded_trajectory_home_pose_topic").as_string().c_str())); // Set the topic on which this planned trajectory will be saved
    moveit_visual_tools.deleteAllMarkers();
    
    // Set the named target based on the position of the desired path relative to the base frame.
    if(cartesian_path_first_pose.pose.position.y < 0.0)
      move_group_interface.setNamedTarget(home_target);
    else
      move_group_interface.setNamedTarget(home_inverse_target);

    //Create a plan to the target pose
    success = move_group_interface.plan(plan); 
    if(!success){
      RCLCPP_ERROR(logger, "Planning failed. Aborting...");
      // Hex is used in these strings to control spacing in RViz
      draw_title(moveit_visual_tools, "Planning\xa0\x66\x61iled!");
      rclcpp::shutdown();
      exit(1);
    }
    draw_planned_trajectory(moveit_visual_tools, move_group_interface, planning_group, plan.trajectory_);
    moveit_visual_tools.trigger();

    // Execute the plan
    if(is_task_space_controller){
      execute_trajectory(moveit_visual_tools, node->get_task_space_trajectory(plan.trajectory_, robot_state, &logger,end_effector_link_name,base_frame, planning_group), node);
    }else{
      execute_trajectory(moveit_visual_tools, plan.trajectory_.joint_trajectory, node);
    }
  }

  /*******************************************************************************
   * 2. PLAN TO THE FIRST POINT OF THE TRAJECTORY
   ******************************************************************************/
  node->set_parameter(rclcpp::Parameter("writer_topic", node->get_parameter("recorded_trajectory_point2point_topic").as_string().c_str())); // Set the topic on which this planned trajectory will be saved

  moveit_visual_tools.deleteAllMarkers();
  draw_cartesian_path(moveit_visual_tools, cartesian_path_poses);
  draw_title(moveit_visual_tools, "Planning");
  move_group_interface.setPoseTarget(cartesian_path_first_pose);
  
  moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to plan to the target pose");
  
  auto current_pose =  move_group_interface.getCurrentPose();
  draw_start_and_target_points(moveit_visual_tools, current_pose, cartesian_path_first_pose);

  

  // if(node->get_parameter("setPoseTarget").as_int() == 0){
  //   move_group_interface.setPoseTarget(cartesian_path_first_pose);
  // }else if(node->get_parameter("setPoseTarget").as_int() == 1){
  //   bool within_bounds = move_group_interface.setJointValueTarget(cartesian_path_first_pose.pose, eef_name);

  //   if (!within_bounds){
  //     RCLCPP_WARN(logger, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  //   }
  // }
  // else{
  //   bool within_bounds = move_group_interface.setApproximateJointValueTarget(cartesian_path_first_pose.pose, eef_name);

  //   if (!within_bounds){
  //     RCLCPP_WARN(logger, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  //   }
  // }
  
  success = move_group_interface.plan(plan);
  if(!success){
    RCLCPP_ERROR(logger, "Planning failed. Aborting...");
    // Hex is used in these strings to control spacing in RViz
    draw_title(moveit_visual_tools, "Planning\xa0\x66\x61iled!");
    rclcpp::shutdown();
    exit(1);
  }
  draw_planned_trajectory(moveit_visual_tools, move_group_interface, planning_group, plan.trajectory_);
  if(!node->plan_validation(plan,cartesian_path_first_pose.pose, robot_state, node->get_parameter("goal_position_tolerance").as_double(), &logger, end_effector_link_name)){
    RCLCPP_ERROR(logger, "Planning failed: the error between desired position and planned position is too big!");
    // Hex is used in these strings to control spacing in RViz
    draw_title(moveit_visual_tools, "Planning\xa0\x66\x61iled!");
    rclcpp::shutdown();
    exit(1);
  }
  

  if(is_task_space_controller){
    execute_trajectory(moveit_visual_tools, node->get_task_space_trajectory(plan.trajectory_, robot_state, &logger,end_effector_link_name,base_frame, planning_group), node);
  }else{
    execute_trajectory(moveit_visual_tools, plan.trajectory_.joint_trajectory, node);
  }

  /*******************************************************************************
   * 3. PLAN THE CARTESIAN PATH
   ******************************************************************************/
  node->set_parameter(rclcpp::Parameter("writer_topic", node->get_parameter("recorded_trajectory_cartesian_topic").as_string().c_str())); // Set the topic on which this planned trajectory will be saved

  //Clean the scene
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.prompt("Press 'Next' in the RvizVisualToolsGui window to plan a Cartesian path");

  draw_title(moveit_visual_tools, "Planning\xa0 cartesian\xa0path");

  //POPULATING WAYPOINTS
  auto const waypoints = populate_waypoints(cartesian_path_poses);

  //CONSTRAINTS' DEFINITION
  moveit_msgs::msg::Constraints constraints;
  moveit_msgs::msg::OrientationConstraint orientation_constraint;
  orientation_constraint.link_name = end_effector_link_name;
  orientation_constraint.header.frame_id = cartesian_path_poses->header.frame_id;
  orientation_constraint.orientation = cartesian_path_poses->poses[0].orientation;
  orientation_constraint.absolute_x_axis_tolerance = 0.01;
  orientation_constraint.absolute_y_axis_tolerance = 0.01;
  orientation_constraint.absolute_z_axis_tolerance = 0.01;
  orientation_constraint.weight = 100.0;
  constraints.orientation_constraints.push_back(orientation_constraint);
  auto [fraction,cartesian_plan] = node->cartesian_path_planning(waypoints, &logger, &move_group_interface, constraints);
  success = fraction == 1.0;
  draw_planned_trajectory(moveit_visual_tools, move_group_interface, planning_group, cartesian_plan.trajectory_);
  if(!success){
    RCLCPP_ERROR(logger, "Planning failed. Aborting...");
    // Hex is used in these strings to control spacing in RViz
    draw_title(moveit_visual_tools, "Planning\xa0\x66\x61iled!");
    rclcpp::shutdown();
    exit(1);
  }
  
  
  // Execute the plan
  
  if(is_task_space_controller){
    execute_trajectory(moveit_visual_tools, node->get_task_space_trajectory(cartesian_plan.trajectory_, robot_state, &logger,end_effector_link_name,base_frame, planning_group), node);
  }else{
    execute_trajectory(moveit_visual_tools, cartesian_plan.trajectory_.joint_trajectory, node);
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();

  return 0;
}