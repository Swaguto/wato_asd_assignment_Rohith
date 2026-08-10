#include <algorithm>
#include <cmath>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  initializeCostmap();
  markObstacles(*scan);
  inflateObstacles();
}

void CostmapCore::initializeCostmap() {
  grid_.assign(static_cast<size_t>(width_ * height_), 0);
}

void CostmapCore::markObstacles(const sensor_msgs::msg::LaserScan& scan) {
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    double range = scan.ranges[i];
    if (range <= scan.range_min || range >= scan.range_max) {
      continue;
    }
    double angle = scan.angle_min + i * scan.angle_increment;

    int x_grid = 0;
    int y_grid = 0;
    convertToGrid(range, angle, x_grid, y_grid);

    if (x_grid < 0 || x_grid >= width_ || y_grid < 0 || y_grid >= height_) {
      continue;
    }
    grid_[static_cast<size_t>(y_grid * width_ + x_grid)] = MAX_COST_;
  }
}

void CostmapCore::convertToGrid(double range, double angle, int& x_grid, int& y_grid) {
  double x = range * std::cos(angle);
  double y = range * std::sin(angle);
  x_grid = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  y_grid = static_cast<int>(std::floor((y - origin_y_) / resolution_));
}

void CostmapCore::inflateObstacles() {
  int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (grid_[static_cast<size_t>(y * width_ + x)] != MAX_COST_) {
        continue;
      }
      int min_x = std::max(0, x - inflation_cells);
      int max_x = std::min(width_ - 1, x + inflation_cells);
      int min_y = std::max(0, y - inflation_cells);
      int max_y = std::min(height_ - 1, y + inflation_cells);
      for (int ny = min_y; ny <= max_y; ++ny) {
        for (int nx = min_x; nx <= max_x; ++nx) {
          double dist = std::hypot((nx - x) * resolution_, (ny - y) * resolution_);
          if (dist >= inflation_radius_) {
            continue;
          }
          int8_t cost = static_cast<int8_t>(MAX_COST_ * (1.0 - dist / inflation_radius_));
          size_t idx = static_cast<size_t>(ny * width_ + nx);
          if (cost > grid_[idx]) {
            grid_[idx] = cost;
          }
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::getCostmapMsg() {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.stamp = rclcpp::Clock().now();
  msg.header.frame_id = "odom";
  msg.info.resolution = resolution_;
  msg.info.width = static_cast<uint32_t>(width_);
  msg.info.height = static_cast<uint32_t>(height_);
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data = grid_;
  return msg;
}

}