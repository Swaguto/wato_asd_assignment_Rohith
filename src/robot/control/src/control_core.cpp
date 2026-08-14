#include <algorithm>
#include <cmath>
#include <limits>

#include "control_core.hpp"

namespace robot
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

// Returns an angle normalized to [-pi, pi].
double wrapAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}
}

ControlCore::ControlCore(const rclcpp::Logger& logger) : logger_(logger) {}

void ControlCore::setParameters(double lookahead_distance, double steering_gain,
                                double max_steering_angle, double linear_velocity) {
  lookahead_distance_ = lookahead_distance;
  steering_gain_ = steering_gain;
  max_steering_angle_ = max_steering_angle;
  linear_velocity_ = linear_velocity;
}

void ControlCore::updatePath(const nav_msgs::msg::Path& path) {
  path_ = path;
}

geometry_msgs::msg::Twist ControlCore::step(double robot_x, double robot_y,
                                            double robot_yaw,
                                            const nav_msgs::msg::OccupancyGrid* local_costmap) const {
  geometry_msgs::msg::Twist twist;

  if (path_.poses.empty()) {
    return twist;
  }

  const int idx = findLookaheadIndex(robot_x, robot_y, robot_yaw);
  const auto& target = path_.poses[static_cast<size_t>(idx)].pose.position;

  const double heading_to_target = std::atan2(target.y - robot_y, target.x - robot_x);
  double steering = wrapAngle(heading_to_target - robot_yaw);

  if (std::abs(steering) > max_steering_angle_) {
    // Keep turning toward the path instead of freezing; push forward only
    // when the path is nearly behind (nothing in the way of a turn).
    if (std::abs(steering) > kPi / 2.0) {
      twist.linear.x = 0.25 * linear_velocity_;
    } else {
      twist.linear.x = 0.0;
    }
  } else {
    // Slow down before sharp turns: at full speed the robot cannot negotiate
    // the maze's 90 degree corners without drifting wide into walls. Scale
    // the speed with how much of the steering range the turn demands.
    const double turn_share = std::abs(steering) / max_steering_angle_;
    twist.linear.x = linear_velocity_ * (1.0 - 0.7 * turn_share * turn_share);
  }

  // Local-costmap protection: never push at speed into an obstacle the lidar
  // can see. The costmap lives in the sensor frame, where +x is always the
  // robot's forward direction, so no pose math is needed here. A blocked
  // wedge directly ahead cuts the command speed to zero; a close hot wedge
  // limits it to a crawl. The planner replans around the wall meanwhile.
  if (local_costmap != nullptr && local_costmap->data.size() >= 16) {
    const double res = local_costmap->info.resolution;
    const int width = static_cast<int>(local_costmap->info.width);
    const double ox = local_costmap->info.origin.position.x;
    for (const double distance : {0.6, 1.0, 1.5}) {
      const int gx = static_cast<int>(std::floor((distance - ox) / res));
      int blocked = 0;
      for (int angle_cell = -2; angle_cell <= 2; ++angle_cell) {
        const int gy = static_cast<int>(
          std::floor((std::tan(angle_cell * 0.2) * distance - ox) / res));
        if (gx < 0 || gx >= width || gy < 0 ||
            gy >= static_cast<int>(local_costmap->info.height)) {
          continue;
        }
        const int8_t cost = local_costmap->data[
          static_cast<size_t>(gy) * width + gx];
        if (cost >= 90) {
          ++blocked;
        }
      }
      if (blocked >= 3 && distance <= 1.0) {
        twist.linear.x = 0.0;
        break;
      }
      if (blocked >= 3) {
        twist.linear.x = std::min(twist.linear.x, 0.25 * linear_velocity_);
      }
    }
  }

  steering = std::clamp(steering, -max_steering_angle_, max_steering_angle_);
  twist.angular.z = steering_gain_ * steering;
  return twist;
}

int ControlCore::findLookaheadIndex(double robot_x, double robot_y,
                                    double robot_yaw) const {
  // Preferred: the closest pose that is at least lookahead_distance away and
  // lies within the forward half-plane. If none qualifies, drive to the end
  // of the path.
  double best_distance = std::numeric_limits<double>::max();
  int best_index = -1;

  for (size_t i = 0; i < path_.poses.size(); ++i) {
    const auto& p = path_.poses[i].pose.position;
    const double dx = p.x - robot_x;
    const double dy = p.y - robot_y;
    const double distance = std::hypot(dx, dy);

    if (distance < lookahead_distance_) {
      continue;
    }
    if (std::abs(wrapAngle(std::atan2(dy, dx) - robot_yaw)) > kPi / 2.0) {
      continue;
    }
    if (distance < best_distance) {
      best_distance = distance;
      best_index = static_cast<int>(i);
    }
  }

  if (best_index < 0) {
    best_index = static_cast<int>(path_.poses.size()) - 1;
  }
  return best_index;
}

}
