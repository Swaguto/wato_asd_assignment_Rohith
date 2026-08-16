#include <algorithm>
#include <cmath>
#include <limits>

#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger),
    lookahead_distance_(1.5),
    max_steering_angle_(0.5),
    steering_gain_(1.5),
    linear_velocity_(1.0),
    goal_tolerance_(1.0) {}

void ControlCore::initControlCore(
  double lookahead_distance,
  double max_steering_angle,
  double steering_gain,
  double linear_velocity,
  double goal_tolerance)
{
  lookahead_distance_ = lookahead_distance;
  max_steering_angle_ = max_steering_angle;
  steering_gain_ = steering_gain;
  linear_velocity_ = linear_velocity;
  goal_tolerance_ = goal_tolerance;
}

void ControlCore::updatePath(const nav_msgs::msg::Path& path) {
  path_ = path;
}

bool ControlCore::isPathEmpty() const {
  return path_.poses.empty();
}

bool ControlCore::isGoalReached(double robot_x, double robot_y) const {
  if (path_.poses.empty()) {
    return true;
  }

  const auto& last = path_.poses.back().pose.position;
  return std::hypot(last.x - robot_x, last.y - robot_y) < goal_tolerance_;
}

geometry_msgs::msg::Twist ControlCore::calculateControlCommand(
  double robot_x,
  double robot_y,
  double robot_theta)
{
  geometry_msgs::msg::Twist cmd;

  if (path_.poses.empty() || isGoalReached(robot_x, robot_y)) {
    return cmd;
  }

  size_t closest_index = 0;
  double closest_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path_.poses.size(); ++i) {
    const auto& p = path_.poses[i].pose.position;
    const double distance = std::hypot(p.x - robot_x, p.y - robot_y);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_index = i;
    }
  }

  const size_t target_index = findLookaheadIndex(closest_index, robot_x, robot_y);
  const auto& target = path_.poses[target_index].pose.position;

  const double angle_to_target = std::atan2(target.y - robot_y, target.x - robot_x);
  double steering = angle_to_target - robot_theta;

  while (steering > M_PI) {
    steering -= 2.0 * M_PI;
  }
  while (steering < -M_PI) {
    steering += 2.0 * M_PI;
  }

  cmd.linear.x = (std::abs(steering) > max_steering_angle_) ? 0.0 : linear_velocity_;

  const double clamped = std::max(-max_steering_angle_, std::min(steering, max_steering_angle_));
  cmd.angular.z = clamped * steering_gain_;

  return cmd;
}

size_t ControlCore::findLookaheadIndex(size_t closest_index, double robot_x, double robot_y) const {

  for (size_t i = closest_index; i < path_.poses.size(); ++i) {
    const auto& p = path_.poses[i].pose.position;
    if (std::hypot(p.x - robot_x, p.y - robot_y) >= lookahead_distance_) {
      return i;
    }
  }

  return path_.poses.size() - 1;
}

}
