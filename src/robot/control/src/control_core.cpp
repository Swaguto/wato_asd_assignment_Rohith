#include <algorithm>
#include <cmath>

#include "control_core.hpp"

namespace robot
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kLinearSpeed = 0.2;
constexpr double kAngularGain = 1.0;
constexpr double kMinDistance = 0.15;
}

ControlCore::ControlCore(const rclcpp::Logger& logger)
: logger_(logger) {}

geometry_msgs::msg::Twist ControlCore::computeTwist(
  const geometry_msgs::msg::PoseStamped& target,
  const nav_msgs::msg::Odometry& odom) const
{
  geometry_msgs::msg::Twist twist;

  const double robot_x = odom.pose.pose.position.x;
  const double robot_y = odom.pose.pose.position.y;
  const auto& q = odom.pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  const double dx = target.pose.position.x - robot_x;
  const double dy = target.pose.position.y - robot_y;
  const double dist = std::hypot(dx, dy);

  double yaw_error = std::atan2(dy, dx) - yaw;
  while (yaw_error > kPi) yaw_error -= 2.0 * kPi;
  while (yaw_error < -kPi) yaw_error += 2.0 * kPi;

  if (dist < kMinDistance) {
    return twist;
  }

  twist.linear.x = kLinearSpeed;
  twist.angular.z = std::clamp(kAngularGain * yaw_error, -1.5, 1.5);
  return twist;
}

}