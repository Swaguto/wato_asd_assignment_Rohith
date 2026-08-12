#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

// Simple pursuit controller: steer toward the next path point that lies at
// least `lookahead_distance` ahead and in front of the robot. Forward speed
// is held constant, but is cut entirely if the required steering angle
// exceeds `max_steering_angle`.
class ControlCore {
  public:
    explicit ControlCore(const rclcpp::Logger& logger);

    void setParameters(double lookahead_distance, double steering_gain,
                       double max_steering_angle, double linear_velocity);
    void updatePath(const nav_msgs::msg::Path& path);

    geometry_msgs::msg::Twist step(double robot_x, double robot_y, double robot_yaw) const;

  private:
    int findLookaheadIndex(double robot_x, double robot_y, double robot_yaw) const;

    nav_msgs::msg::Path path_;
    rclcpp::Logger logger_;

    double lookahead_distance_ = 1.5;
    double steering_gain_ = 1.5;
    double max_steering_angle_ = 0.5;
    double linear_velocity_ = 1.0;
};

}

#endif
