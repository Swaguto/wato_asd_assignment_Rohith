#include <algorithm>
#include <cmath>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
  : costmap_data_(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
    logger_(logger),
    inflation_radius_(1.0),
    inflation_cells_(0) {}

void CostmapCore::initCostmap(
  double resolution,
  int width,
  int height,
  const geometry_msgs::msg::Pose& origin,
  double inflation_radius)
{
  costmap_data_->info.resolution = resolution;
  costmap_data_->info.width = width;
  costmap_data_->info.height = height;
  costmap_data_->info.origin = origin;
  costmap_data_->data.assign(width * height, -1);

  inflation_radius_ = inflation_radius;
  inflation_cells_ = static_cast<int>(std::ceil(inflation_radius / resolution));

  RCLCPP_INFO(
    logger_, "Costmap initialized: %.2f m/cell, %dx%d cells, inflation radius %.2f m",
    resolution, width, height, inflation_radius);
}

void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan) {

  std::fill(costmap_data_->data.begin(), costmap_data_->data.end(), -1);

  const double res = costmap_data_->info.resolution;
  const double origin_x = costmap_data_->info.origin.position.x;
  const double origin_y = costmap_data_->info.origin.position.y;
  const int width = static_cast<int>(costmap_data_->info.width);
  const int height = static_cast<int>(costmap_data_->info.height);

  double angle = scan->angle_min;
  for (size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment) {
    const double range = scan->ranges[i];

    if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
      continue;
    }

    const double x = range * std::cos(angle);
    const double y = range * std::sin(angle);

    const int hit_x = static_cast<int>((x - origin_x) / res);
    const int hit_y = static_cast<int>((y - origin_y) / res);

    if (hit_x < 0 || hit_x >= width || hit_y < 0 || hit_y >= height) {
      continue;
    }

    const double step = res * 0.5;
    for (double d = step; d < range; d += step) {
      const double rx = d * std::cos(angle);
      const double ry = d * std::sin(angle);
      const int free_x = static_cast<int>((rx - origin_x) / res);
      const int free_y = static_cast<int>((ry - origin_y) / res);
      if (free_x < 0 || free_x >= width || free_y < 0 || free_y >= height) {
        break;
      }
      int8_t& free_cell = costmap_data_->data[free_y * width + free_x];
      if (free_cell < 0) {
        free_cell = 0;
      }
    }

    costmap_data_->data[hit_y * width + hit_x] = 100;
    inflateObstacle(hit_x, hit_y);
  }
}

void CostmapCore::inflateObstacle(int cell_x, int cell_y) {
  const int width = static_cast<int>(costmap_data_->info.width);
  const int height = static_cast<int>(costmap_data_->info.height);
  const double res = costmap_data_->info.resolution;

  const int min_x = std::max(0, cell_x - inflation_cells_);
  const int max_x = std::min(width - 1, cell_x + inflation_cells_);
  const int min_y = std::max(0, cell_y - inflation_cells_);
  const int max_y = std::min(height - 1, cell_y + inflation_cells_);

  for (int y = min_y; y <= max_y; ++y) {
    for (int x = min_x; x <= max_x; ++x) {
      const double distance = std::hypot(x - cell_x, y - cell_y) * res;
      if (distance > inflation_radius_) {
        continue;
      }

      const int8_t cost = static_cast<int8_t>(100.0 * (1.0 - distance / inflation_radius_));
      int8_t& cell = costmap_data_->data[y * width + x];
      cell = std::max(cell, cost);
    }
  }
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmapData() const {
  return costmap_data_;
}

}
