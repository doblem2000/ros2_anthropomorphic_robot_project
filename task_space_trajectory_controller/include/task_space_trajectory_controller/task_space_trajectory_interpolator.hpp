#ifndef TASK_SPACE_TRAJECTORY_INTERPOLATOR_H_INCLUDED
#define TASK_SPACE_TRAJECTORY_INTERPOLATOR_H_INCLUDED

#include "moveit_msgs/msg/cartesian_trajectory.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"

#include "moveit_msgs/msg/cartesian_trajectory.hpp"

namespace task_space_trajectory_controller
{

using TrajectoryPointConstIter = std::vector<moveit_msgs::msg::CartesianTrajectoryPoint>::const_iterator;

enum class InterpolationMethod
{
  NONE = 0,
  LINEAR = 1,
};

/**
 * @class TaskSpaceTrajectoryInterpolator
 * @brief Class for interpolating a task space trajectory.
 */
class TaskSpaceTrajectoryInterpolator {
public:
  /**
   * @brief Default constructor.
   */
  TaskSpaceTrajectoryInterpolator();

  /**
   * @brief Destructor.
   */
  virtual ~TaskSpaceTrajectoryInterpolator() = default;

  /**
   * @brief Set the trajectory to be interpolated.
   * @param[in] trajectory_msg The trajectory message.
   * @param[in] current_point The initial state before the trajectory is executed.
   */
  void set_trajectory(const std::shared_ptr<moveit_msgs::msg::CartesianTrajectory> trajectory_msg, std::shared_ptr<moveit_msgs::msg::CartesianTrajectoryPoint> current_point);

  /**
   * @brief Get a pointer to the trajectory.
   * @return A shared pointer to the trajectory message. [out]
   */
  std::shared_ptr<moveit_msgs::msg::CartesianTrajectory> get_trajectory_ptr() const;

  /**
   * @brief Set the interpolation method.
   * @param[in] interpolation_method The interpolation method to be used.
   */
  void set_interpolation_method(InterpolationMethod interpolation_method);

  /**
   * @brief Sample a point from the trajectory at a given time.
   * @param[in] sample_time The time at which to sample the trajectory.
   * @param[out] sampled_point The sampled point from the trajectory.
   * @param[out] start_segment_itr Iterator pointing to the start segment of the trajectory.
   * @param[out] end_segment_itr Iterator pointing to the end segment of the trajectory.
   * @param[in] logger  The logger for logging messages.
   * @return True if sampling is successful, false otherwise.
   */
  bool sample(const rclcpp::Time &sample_time, std::shared_ptr<moveit_msgs::msg::CartesianTrajectoryPoint> sampled_point,
              TrajectoryPointConstIter &start_segment_itr, TrajectoryPointConstIter &end_segment_itr, const rclcpp::Logger &logger);

  /**
   * @brief Interpolate between two points in the trajectory.
   * @details The interpolation method is specified by the interpolation_method_ member variable.
   *          If the interpolation method is NONE, the end point is returned. If the interpolation method is LINEAR,
   *          a linear interpolation of position, orientation and velocity is performed. The acceleration is ignored.
   *          The interpolation method used for orientation is SLERP.
   * @param[in] start_point The start point of the interpolation.
   * @param[in] end_point The end point of the interpolation.
   * @param[in] time_from_start The time from the start of the trajectory at which to interpolate between the two points.
   * @param[out] sampled_point The interpolated point.
   * @param[in] logger The logger for logging messages.
   */
  void interpolate_between_points(const moveit_msgs::msg::CartesianTrajectoryPoint &start_point, const moveit_msgs::msg::CartesianTrajectoryPoint &end_point,
                                  const rclcpp::Duration &time_from_start, moveit_msgs::msg::CartesianTrajectoryPoint &sampled_point, const rclcpp::Logger &logger);

  /**
   * @brief Get an iterator pointing to the beginning of the trajectory.
   * @return The iterator pointing to the beginning of the trajectory.
   */
  TrajectoryPointConstIter begin() const;

  /**
   * @brief Get an iterator pointing to the end of the trajectory.
   * @return The iterator pointing to the end of the trajectory.
   */
  TrajectoryPointConstIter end() const;

private:
  std::shared_ptr<moveit_msgs::msg::CartesianTrajectory> trajectory_msg_;      ///< The trajectory message.
  std::shared_ptr<moveit_msgs::msg::CartesianTrajectoryPoint> starting_point_; ///< The initial state before the trajectory is executed.
  TrajectoryPointConstIter current_point_itr_;                                 ///< Iterator pointing to the current point in the trajectory.
  InterpolationMethod interpolation_method_;                                   ///< The interpolation method.
  rclcpp::Time trajectory_start_time_;                                         ///< The start time of the trajectory.
  bool was_trajectory_sampled_;                                                ///< Flag indicating whether the trajectory was sampled.
};
}

#endif // TASK_SPACE_TRAJECTORY_CONTROLLER_H_INCLUDED