#include <algorithm>
#include <cmath>

#include "map_memory_core.hpp"

namespace robot
{

namespace
{
constexpr int kGridWidth = 500;
constexpr int kGridHeight = 500;
constexpr double kGridOriginX = -25.0;
constexpr double kGridOriginY = -25.0;
constexpr int kObstacleThreshold = 50;
}

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
: logger_(logger) {}

void MapMemoryCore::initialize(const nav_msgs::msg::OccupancyGrid& sample)
{
  global_map_.header.frame_id = "map";
  global_map_.info.resolution = sample.info.resolution;
  global_map_.info.width = kGridWidth;
  global_map_.info.height = kGridHeight;
  global_map_.info.origin.position.x = kGridOriginX;
  global_map_.info.origin.position.y = kGridOriginY;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(kGridWidth * kGridHeight, 0);
  initialized_ = true;
}

void MapMemoryCore::mergeCostmap(const nav_msgs::msg::OccupancyGrid& local,
                                 const nav_msgs::msg::Odometry& odom)
{
  const double robot_x = odom.pose.pose.position.x;
  const double robot_y = odom.pose.pose.position.y;
  const auto& q = odom.pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const double l_res = local.info.resolution;
  const int l_width = static_cast<int>(local.info.width);

  for (int i = 0; i < static_cast<int>(local.data.size()); ++i) {
    if (local.data[i] < kObstacleThreshold) {
      continue;
    }
    const int local_row = i / l_width;
    const int local_col = i % l_width;
    const double local_x = local.info.origin.position.x + (local_col + 0.5) * l_res;
    const double local_y = local.info.origin.position.y + (local_row + 0.5) * l_res;

    const double global_x = robot_x + local_x * cos_yaw - local_y * sin_yaw;
    const double global_y = robot_y + local_x * sin_yaw + local_y * cos_yaw;

    const int global_col = static_cast<int>(std::floor(
      (global_x - global_map_.info.origin.position.x) / global_map_.info.resolution));
    const int global_row = static_cast<int>(std::floor(
      (global_y - global_map_.info.origin.position.y) / global_map_.info.resolution));

    if (global_row < 0 || global_row >= static_cast<int>(global_map_.info.height) ||
        global_col < 0 || global_col >= static_cast<int>(global_map_.info.width)) {
      continue;
    }

    const size_t global_idx = global_row * global_map_.info.width + global_col;
    global_map_.data[global_idx] = std::max(global_map_.data[global_idx], local.data[i]);
  }
}

void MapMemoryCore::updateMap(const nav_msgs::msg::OccupancyGrid& costmap,
                              const nav_msgs::msg::Odometry& odom)
{
  if (!initialized_) {
    initialize(costmap);
  }
  mergeCostmap(costmap, odom);
}

const nav_msgs::msg::OccupancyGrid& MapMemoryCore::getMap() const
{
  return global_map_;
}

}