#include "interactive_markers/interactive_marker_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "moveit_msgs/msg/cartesian_trajectory_point.hpp"

#include "reference_generator_parameters.hpp"

void processFeedback(const rclcpp::Node::SharedPtr &node, const interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr &feedback, const rclcpp::Publisher<moveit_msgs::msg::CartesianTrajectoryPoint>::SharedPtr &reference_publisher){
  //RCLCPP_INFO(node->get_logger(), "Reference frame moved to position (%f, %f, %f)", feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
  //RCLCPP_INFO(node->get_logger(), "Reference frame moved to orientation (%f, %f, %f, %f)", feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w);
  moveit_msgs::msg::CartesianTrajectoryPoint reference;
  reference.point.pose.position.x = feedback->pose.position.x;
  reference.point.pose.position.y = feedback->pose.position.y;
  reference.point.pose.position.z = feedback->pose.position.z;
  reference.point.pose.orientation.x = feedback->pose.orientation.x;
  reference.point.pose.orientation.y = feedback->pose.orientation.y;
  reference.point.pose.orientation.z = feedback->pose.orientation.z;
  reference.point.pose.orientation.w = feedback->pose.orientation.w;
  reference.point.velocity.linear.x = 0.0;
  reference.point.velocity.linear.y = 0.0;
  reference.point.velocity.linear.z = 0.0;
  reference.point.velocity.angular.x = 0.0;
  reference.point.velocity.angular.y = 0.0;
  reference.point.velocity.angular.z = 0.0;
  reference_publisher->publish(reference);
}

void make_6DOF_control(visualization_msgs::msg::InteractiveMarker &int_marker){
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
}

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("reference_generator");

  std::thread t([&node](){
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  });

  auto param_listener = std::make_shared<reference_generator::ParamListener>(node);
  auto params = param_listener->get_params();

  auto interactive_marker_server = std::make_shared<interactive_markers::InteractiveMarkerServer>("reference_generator", node);
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node, true);
  auto reference_publisher = node->create_publisher<moveit_msgs::msg::CartesianTrajectoryPoint>(params.command_topic, 10);

  // while(!tf_buffer->canTransform(params.base_link, params.end_effector_link, tf2::TimePointZero, tf2::durationFromSec(1.0))){
  //   RCLCPP_INFO(node->get_logger(), "Waiting for transform from %s to %s", params.base_link.c_str(), params.end_effector_link.c_str());
  //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // }
  
  auto end_effector_transform = tf_buffer->lookupTransform(params.base_link, params.end_effector_link, tf2::TimePointZero, tf2::durationFromSec(10.0));
  RCLCPP_INFO(node->get_logger(), "Transform from %s to %s: %f %f %f %f %f %f %f", params.base_link.c_str(), params.end_effector_link.c_str(), end_effector_transform.transform.translation.x, end_effector_transform.transform.translation.y, end_effector_transform.transform.translation.z, end_effector_transform.transform.rotation.x, end_effector_transform.transform.rotation.y, end_effector_transform.transform.rotation.z, end_effector_transform.transform.rotation.w);
  
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = params.base_link;
  int_marker.header.stamp = node->now();
  int_marker.name = "reference_frame";
  int_marker.description = "Reference frame";
  int_marker.scale = 0.5;
  int_marker.pose.position.x = end_effector_transform.transform.translation.x;
  int_marker.pose.position.y = end_effector_transform.transform.translation.y;
  int_marker.pose.position.z = end_effector_transform.transform.translation.z;
  int_marker.pose.orientation.x = end_effector_transform.transform.rotation.x;
  int_marker.pose.orientation.y = end_effector_transform.transform.rotation.y;
  int_marker.pose.orientation.z = end_effector_transform.transform.rotation.z;
  int_marker.pose.orientation.w = end_effector_transform.transform.rotation.w;

  make_6DOF_control(int_marker);

  interactive_marker_server->insert(int_marker);
  interactive_marker_server->setCallback(int_marker.name, [&node, &reference_publisher](const interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr &feedback){
    processFeedback(node, feedback, reference_publisher);
  });
  interactive_marker_server->applyChanges();

  t.join();
  
  rclcpp::shutdown();
  return 0;
}