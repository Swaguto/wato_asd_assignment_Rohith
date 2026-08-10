#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace robot
{

class ControlCore {
  public:
    ControlCore(const rclcpp::Logger& logger);

    geometry_msgs::msg::Twist computeTwist(const geometry_msgs::msg::PoseStamped& target,
                                           const nav_msgs::msg::Odometry& odom) const;

  private:
    rclcpp::Logger logger_;
};

}

#endif