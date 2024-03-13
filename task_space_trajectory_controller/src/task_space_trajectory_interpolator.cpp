#include "task_space_trajectory_controller/task_space_trajectory_interpolator.hpp"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "hardware_interface/macros.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/logging.hpp"
#include "std_msgs/msg/header.hpp"

namespace task_space_trajectory_controller
{

TaskSpaceTrajectoryInterpolator::TaskSpaceTrajectoryInterpolator() : trajectory_msg_(nullptr),
                                                                      interpolation_method_(InterpolationMethod::NONE), trajectory_start_time_(0), was_trajectory_sampled_(false){
}

void TaskSpaceTrajectoryInterpolator::set_trajectory(const std::shared_ptr<moveit_msgs::msg::CartesianTrajectory> trajectory_msg, std::shared_ptr<moveit_msgs::msg::CartesianTrajectoryPoint> current_point){
  THROW_ON_NULLPTR(trajectory_msg);
  THROW_ON_NULLPTR(current_point);
  trajectory_msg_ = trajectory_msg;
  starting_point_ = current_point;
  // Ensure that the first trajectory point has a time_from_start of 0
  starting_point_->time_from_start = rclcpp::Duration(0, 0);
  // Set the trajectory start time to header stamp of the trajectory message
  trajectory_start_time_ = trajectory_msg_->header.stamp;
  // Set the current point iterator to the first point of the trajectory
  current_point_itr_ = begin();
  was_trajectory_sampled_ = false;
}

std::shared_ptr<moveit_msgs::msg::CartesianTrajectory> TaskSpaceTrajectoryInterpolator::get_trajectory_ptr() const{
  return trajectory_msg_;
}

void TaskSpaceTrajectoryInterpolator::set_interpolation_method(InterpolationMethod interpolation_method){
  interpolation_method_ = interpolation_method;
}

bool TaskSpaceTrajectoryInterpolator::sample(const rclcpp::Time &sample_time, std::shared_ptr<moveit_msgs::msg::CartesianTrajectoryPoint> sampled_point,
                                              TrajectoryPointConstIter &start_segment_itr, TrajectoryPointConstIter &end_segment_itr, const rclcpp::Logger & logger){

  THROW_ON_NULLPTR(trajectory_msg_);
  THROW_ON_NULLPTR(sampled_point);

  // If the trajectory message is empty, return false
  if (trajectory_msg_->points.empty()){
    start_segment_itr = end();
    end_segment_itr = end();
    current_point_itr_ = end();
    return false;
  }

  if (!was_trajectory_sampled_){
    // If the trajectory start time is not set, set it to the first sampling time
    if (trajectory_start_time_.seconds() == 0.0)
    {
      trajectory_start_time_ = sample_time;
    }
    was_trajectory_sampled_ = true;
  }

  // If the sampling time is before the trajectory start time, return false
  if (sample_time < trajectory_start_time_){
    start_segment_itr = begin();
    end_segment_itr = begin();
    current_point_itr_ = begin();
    return false;
  }

  const auto &first_point = *begin();
  const rclcpp::Time first_point_timestamp = trajectory_start_time_ + first_point.time_from_start;
  
  // If the sampling time is before the first trajectory point, we need to interpolate between the starting point and the first trajectory point
  if(sample_time < first_point_timestamp){
    interpolate_between_points(*starting_point_, first_point, sample_time - trajectory_start_time_, *sampled_point, logger);
    start_segment_itr = begin();
    end_segment_itr = begin();
    current_point_itr_ = begin();
    return true;
  }

  // We need to find the segment of the trajectory that contains the sampling time
  while(current_point_itr_ != end() && sample_time >= trajectory_start_time_ + current_point_itr_->time_from_start){
    current_point_itr_++;
  }
  // Now current_point_itr_ points to the first point in the trajectory after the sampling time

  // If we have reached the end of the trajectory, return last point
  if(current_point_itr_ == end()){
    start_segment_itr = --end();
    end_segment_itr = end();
    *sampled_point = *start_segment_itr; // Last point of the trajectory
    return true;
  }

  // If we are between two points of the trajectory, we must interpolate between the current point and the next point
  // Notice that:
  // - current_point_itr_ - 1 is guaranteed to be a valid iterator because trajectory_msg_->points is not empty
  // - current_point_itr_ is guaranteed to be a valid iterator because we have checked that current_point_itr_ != end()
  start_segment_itr = current_point_itr_ - 1; // Iterator to the start segment for given sample_time
  end_segment_itr = current_point_itr_; // Iterator to the end segment for given sample_time
  interpolate_between_points(*start_segment_itr, *end_segment_itr, sample_time - trajectory_start_time_, *sampled_point, logger);
  return true;
}

void TaskSpaceTrajectoryInterpolator::interpolate_between_points(const moveit_msgs::msg::CartesianTrajectoryPoint & start_point, const moveit_msgs::msg::CartesianTrajectoryPoint & end_point,
      const rclcpp::Duration & sample_point_time, moveit_msgs::msg::CartesianTrajectoryPoint & sampled_point, const rclcpp::Logger & logger){

  if(interpolation_method_ == InterpolationMethod::NONE){
    sampled_point.time_from_start = sample_point_time;
    sampled_point.point = end_point.point;
    return;
  }

  if(interpolation_method_ == InterpolationMethod::LINEAR){
    // Calculate the percentage of time between the start and end points
    rclcpp::Duration start_point_time(start_point.time_from_start);
    rclcpp::Duration end_point_time(end_point.time_from_start);
    const rcl_duration_value_t duration_between_points = (end_point_time - start_point_time).nanoseconds();
    const rcl_duration_value_t duration_sample = (sample_point_time - start_point_time).nanoseconds();
    const double alpha = static_cast<double>(duration_sample) / static_cast<double>(duration_between_points);
    //RCLCPP_INFO(logger, "duration_between_points: %ld duration_from_start: %ld alpha: %f", duration_between_points, duration_sample, alpha);
    // Interpolate the position
    sampled_point.time_from_start = sample_point_time;
    sampled_point.point.pose.position.x = start_point.point.pose.position.x + alpha * (end_point.point.pose.position.x - start_point.point.pose.position.x);
    sampled_point.point.pose.position.y = start_point.point.pose.position.y + alpha * (end_point.point.pose.position.y - start_point.point.pose.position.y);
    sampled_point.point.pose.position.z = start_point.point.pose.position.z + alpha * (end_point.point.pose.position.z - start_point.point.pose.position.z);

    // Interpolate the orientation (slerp)
    Eigen::Quaterniond start_orientation(start_point.point.pose.orientation.w, start_point.point.pose.orientation.x, start_point.point.pose.orientation.y, start_point.point.pose.orientation.z);
    Eigen::Quaterniond end_orientation(end_point.point.pose.orientation.w, end_point.point.pose.orientation.x, end_point.point.pose.orientation.y, end_point.point.pose.orientation.z);
    Eigen::Quaterniond sampled_orientation = start_orientation.slerp(alpha, end_orientation);
    sampled_point.point.pose.orientation.w = sampled_orientation.w();
    sampled_point.point.pose.orientation.x = sampled_orientation.x();
    sampled_point.point.pose.orientation.y = sampled_orientation.y();
    sampled_point.point.pose.orientation.z = sampled_orientation.z();

    // Interpolate the linear velocity
    sampled_point.point.velocity.linear.x = start_point.point.velocity.linear.x + alpha * (end_point.point.velocity.linear.x - start_point.point.velocity.linear.x);
    sampled_point.point.velocity.linear.y = start_point.point.velocity.linear.y + alpha * (end_point.point.velocity.linear.y - start_point.point.velocity.linear.y);
    sampled_point.point.velocity.linear.z = start_point.point.velocity.linear.z + alpha * (end_point.point.velocity.linear.z - start_point.point.velocity.linear.z);

    // Interpolate the angular velocity
    sampled_point.point.velocity.angular.x = start_point.point.velocity.angular.x + alpha * (end_point.point.velocity.angular.x - start_point.point.velocity.angular.x);
    sampled_point.point.velocity.angular.y = start_point.point.velocity.angular.y + alpha * (end_point.point.velocity.angular.y - start_point.point.velocity.angular.y);
    sampled_point.point.velocity.angular.z = start_point.point.velocity.angular.z + alpha * (end_point.point.velocity.angular.z - start_point.point.velocity.angular.z);

    // TODO: Interpolate the linear acceleration
    return;
  }

}

TrajectoryPointConstIter TaskSpaceTrajectoryInterpolator::begin() const
{
  THROW_ON_NULLPTR(trajectory_msg_)

  return trajectory_msg_->points.begin();
}

TrajectoryPointConstIter TaskSpaceTrajectoryInterpolator::end() const
{
  THROW_ON_NULLPTR(trajectory_msg_)

  return trajectory_msg_->points.end();
}
}