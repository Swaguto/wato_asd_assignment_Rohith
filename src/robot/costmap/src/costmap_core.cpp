#include <algorithm>
#include <cmath>
#include <cstddef>

#include "costmap_core.hpp"

namespace robot
{

namespace
{
constexpr int8_t kOccupiedCost = 100;   // full-cost cell: an obstacle hit
constexpr int8_t kSeedCost = 90;        // cells at/above this value restart inflation
}

CostmapCore::CostmapCore(rclcpp::Node* node)
: logger_(node != nullptr ? node->get_logger()
                          : rclcpp::get_logger("costmap_core"))
{
  if (node == nullptr) {
    return;
  }
  readParameters(node);
  cells_.assign(static_cast<size_t>(width_) * height_, 0);
  configured_ = true;
  RCLCPP_INFO(logger_, "costmap ready: %dx%d cells @ %.2f m, origin (%.2f, %.2f), "
               "inflation %.2f m", width_, height_, resolution_,
               origin_x_, origin_y_, inflation_radius_);
}

void CostmapCore::readParameters(rclcpp::Node* node) {
  node->declare_parameter("costmap.resolution", 0.4);
  node->declare_parameter("costmap.width", 120);
  node->declare_parameter("costmap.height", 120);
  node->declare_parameter("costmap.origin.position.x", -24.0);
  node->declare_parameter("costmap.origin.position.y", -24.0);
  node->declare_parameter("costmap.inflation_radius", 1.5);

  resolution_ = node->get_parameter("costmap.resolution").as_double();
  width_ = node->get_parameter("costmap.width").as_int();
  height_ = node->get_parameter("costmap.height").as_int();
  origin_x_ = node->get_parameter("costmap.origin.position.x").as_double();
  origin_y_ = node->get_parameter("costmap.origin.position.y").as_double();
  inflation_radius_ = node->get_parameter("costmap.inflation_radius").as_double();
}

void CostmapCore::update(const sensor_msgs::msg::LaserScan& scan) {
  if (!configured_) {
    return;
  }

  // Fresh map every scan: nothing from the previous reading carries over.
  std::fill(cells_.begin(), cells_.end(), 0);

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    // NaN and out-of-range samples are not usable hits.
    if (!(range >= scan.range_min && range <= scan.range_max)) {
      continue;
    }
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double x = range * std::cos(angle);
    const double y = range * std::sin(angle);

    const int gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    const int gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
      continue;
    }
    cells_[static_cast<size_t>(gy) * width_ + gx] = kOccupiedCost;
  }

  inflate();
}

void CostmapCore::inflate() {
  // For each occupied cell, spread a gradient (100 at the hit, falling
  // linearly to 0 at the inflation radius) over all reachable neighbours,
  // keeping the highest value from any hit. Queue-driven so the falloff is
  // smooth, like the reference's BFS inflation.
  struct FrontierEntry {
    int x;
    int y;
    int seed_x;
    int seed_y;
  };

  std::vector<FrontierEntry> frontier;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (cells_[static_cast<size_t>(y) * width_ + x] >= kSeedCost) {
        frontier.push_back({x, y, x, y});
      }
    }
  }

  while (!frontier.empty()) {
    const FrontierEntry entry = frontier.back();
    frontier.pop_back();

    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const int nx = entry.x + dx;
        const int ny = entry.y + dy;
        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
          continue;
        }
        const double distance =
          std::hypot(nx - entry.seed_x, ny - entry.seed_y) * resolution_;
        if (distance > inflation_radius_) {
          continue;
        }
        const int8_t value = static_cast<int8_t>(
          (1.0 - distance / inflation_radius_) * kOccupiedCost);
        int8_t& cell = cells_[static_cast<size_t>(ny) * width_ + nx];
        if (value > cell) {
          cell = value;
          frontier.push_back({nx, ny, entry.seed_x, entry.seed_y});
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::buildMessage() const {
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = resolution_;
  msg.info.width = static_cast<uint32_t>(width_);
  msg.info.height = static_cast<uint32_t>(height_);
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.orientation.w = 1.0;
  msg.data = cells_;
  return msg;
}

}
