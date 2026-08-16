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

    void initControlCore(
      double lookahead_distance,
      double max_steering_angle,
      double steering_gain,
      double linear_velocity,
      double goal_tolerance);

    void updatePath(const nav_msgs::msg::Path& path);

    bool isPathEmpty() const;

    bool isGoalReached(double robot_x, double robot_y) const;

    geometry_msgs::msg::Twist calculateControlCommand(
      double robot_x,
      double robot_y,
      double robot_theta);

  private:

    size_t findLookaheadIndex(size_t closest_index, double robot_x, double robot_y) const;

    nav_msgs::msg::Path path_;
    rclcpp::Logger logger_;

    double lookahead_distance_;
    double max_steering_angle_;
    double steering_gain_;
    double linear_velocity_;
    double goal_tolerance_;
};

}

#endif
