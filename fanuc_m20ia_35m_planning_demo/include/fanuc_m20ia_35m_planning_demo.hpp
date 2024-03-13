#ifndef FANUC_M20IA_35M_PLANNING_NODE
#define FANUC_M20IA_35M_PLANNING_NODE

#include <thread>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <filesystem>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "rclcpp/serialization.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "moveit_msgs/msg/cartesian_trajectory.hpp"
#include "planning_node.hpp"

using namespace std::chrono_literals;

/*!
 * \brief Read a path from a bag file and return it as a shared pointer to a PoseArray object. 
  *@param logger: the logger object
  *@param bag_filename: the name of the bag file
  *@param topic_name: the name of the topic
  *@return the path read from the bag file as a shared pointer to a PoseArray
 */
const std::shared_ptr<geometry_msgs::msg::PoseArray> read_cartesian_path_poses(const rclcpp::Logger *logger, std::string bag_filename, std::string topic_name);

/*!
  * \brief Populate a vector of waypoints from a PoseArray.
  * @param cartesian_path_poses: the PoseArray object containing the path
  * @return the Pose vector of waypoints
  * 
*/
const std::vector<geometry_msgs::msg::Pose> populate_waypoints(const std::shared_ptr<geometry_msgs::msg::PoseArray> cartesian_path_poses);

/*!
  * \brief Draw a title in RViz and trigger the rendering. It positions the text 1.5m above the base link.
  * @param moveit_visual_tools: the MoveItVisualTools object
  * @param text: the text to be drawn
  * 
*/
void draw_title(moveit_visual_tools::MoveItVisualTools &moveit_visual_tools, std::string text);

/*!
  * \brief Draw the path in RViz and trigger the rendering.
  * @param moveit_visual_tools: the MoveItVisualTools object
  * @param path_poses: the path to be drawn
  * 
*/
void draw_cartesian_path(moveit_visual_tools::MoveItVisualTools &moveit_visual_tools, const std::shared_ptr<geometry_msgs::msg::PoseArray> path_poses);

/*! 
  * \brief Draw the tool's planned trajectory in RViz and trigger the rendering.
  * @param moveit_visual_tools: the MoveItVisualTools object
  * @param move_group_interface: the MoveGroupInterface object
  * @param planning_group: the name of the planning group
  * @param trajectory: the trajectory to be drawn
  *
*/
void draw_planned_trajectory(moveit_visual_tools::MoveItVisualTools &moveit_visual_tools, moveit::planning_interface::MoveGroupInterface &move_group_interface, const std::string &planning_group, const moveit_msgs::msg::RobotTrajectory &trajectory);

/*!
  * \brief Draw the start and target points in RViz and trigger the rendering.
  * @param moveit_visual_tools: the MoveItVisualTools object
  * @param start_pose: the start point
  * @param target_pose: the target point
  * 
*/
void draw_start_and_target_points(moveit_visual_tools::MoveItVisualTools &moveit_visual_tools, const geometry_msgs::msg::PoseStamped &start_pose, const geometry_msgs::msg::PoseStamped &target_pose);

#endif