#include <algorithm>
#include <cmath>
#include <cstddef>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(rclcpp::Node* node)
: logger_(node != nullptr ? node->get_logger()
                          : rclcpp::get_logger("map_memory_core"))
{
  if (node == nullptr) {
    return;
  }
  readParameters(node);

  map_msg_.header.frame_id = "sim_world";
  map_msg_.info.resolution = resolution_;
  map_msg_.info.width = static_cast<uint32_t>(width_);
  map_msg_.info.height = static_cast<uint32_t>(height_);
  map_msg_.info.origin.position.x = origin_x_;
  map_msg_.info.origin.position.y = origin_y_;
  map_msg_.info.origin.orientation.w = 1.0;
  map_msg_.data.assign(static_cast<size_t>(width_) * height_, 0);

  initialized_ = true;
  RCLCPP_INFO(logger_, "global map ready: %dx%d cells @ %.2f m, origin (%.2f, %.2f)",
              width_, height_, resolution_, origin_x_, origin_y_);
}

void MapMemoryCore::readParameters(rclcpp::Node* node) {
  node->declare_parameter("global_map.resolution", 0.5);
  node->declare_parameter("global_map.width", 60);
  node->declare_parameter("global_map.height", 60);
  node->declare_parameter("global_map.origin.position.x", -15.0);
  node->declare_parameter("global_map.origin.position.y", -15.0);

  resolution_ = node->get_parameter("global_map.resolution").as_double();
  width_ = node->get_parameter("global_map.width").as_int();
  height_ = node->get_parameter("global_map.height").as_int();
  origin_x_ = node->get_parameter("global_map.origin.position.x").as_double();
  origin_y_ = node->get_parameter("global_map.origin.position.y").as_double();
}

void MapMemoryCore::merge(const nav_msgs::msg::OccupancyGrid& local,
                          double robot_x, double robot_y, double robot_yaw) {
  if (!initialized_) {
    return;
  }

  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);
  const double local_res = local.info.resolution;
  const int local_width = static_cast<int>(local.info.width);
  const int local_height = static_cast<int>(local.info.height);
  const double local_ox = local.info.origin.position.x;
  const double local_oy = local.info.origin.position.y;
  const int global_width = static_cast<int>(map_msg_.info.width);
  const int global_height = static_cast<int>(map_msg_.info.height);

  for (int j = 0; j < local_height; ++j) {
    for (int i = 0; i < local_width; ++i) {
      const int8_t value = local.data[static_cast<size_t>(j) * local_width + i];
      if (value < 0) {
        // Never seen: leave the memory alone.
        continue;
      }

      // Cell centre in the sensor frame...
      const double lx = local_ox + (static_cast<double>(i) + 0.5) * local_res;
      const double ly = local_oy + (static_cast<double>(j) + 0.5) * local_res;
      // ...then rigid transform into the world frame.
      const double wx = robot_x + lx * cos_yaw - ly * sin_yaw;
      const double wy = robot_y + lx * sin_yaw + ly * cos_yaw;

      const int gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
      const int gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
      if (gx < 0 || gx >= global_width || gy < 0 || gy >= global_height) {
        continue;
      }

      int8_t& stored = map_msg_.data[static_cast<size_t>(gy) * global_width + gx];
      stored = std::max(stored, value);
    }
  }
}

}
