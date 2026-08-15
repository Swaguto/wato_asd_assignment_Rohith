#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    explicit ControlCore(const rclcpp::Logger& logger);

    // Configure the pure pursuit controller from params.yaml
    void initControlCore(
      double lookahead_distance,
      double max_steering_angle,
      double steering_gain,
      double linear_velocity,
      double goal_tolerance);

    // Store the newest path from the planner
    void updatePath(const nav_msgs::msg::Path& path);

    // True if no path has been received yet
    bool isPathEmpty() const;

    // True if the robot is within the goal tolerance of the path's end
    bool isGoalReached(double robot_x, double robot_y) const;

    // Pure pursuit: pick a lookahead point on the path and steer towards it.
    // Returns a zero twist when there is nothing to follow.
    geometry_msgs::msg::Twist calculateControlCommand(
      double robot_x,
      double robot_y,
      double robot_theta);

  private:
    // Walk forward along the path from the closest point until a point at
    // least lookahead_distance away is found; fall back to the final waypoint
    size_t findLookaheadIndex(size_t closest_index, double robot_x, double robot_y) const;

    nav_msgs::msg::Path path_;
    rclcpp::Logger logger_;

    double lookahead_distance_;
    double max_steering_angle_;
    double steering_gain_;
    double linear_velocity_;
    double goal_tolerance_;
};

}  // namespace robot

#endif