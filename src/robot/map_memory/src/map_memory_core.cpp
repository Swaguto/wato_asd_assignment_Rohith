#include <algorithm>
#include <cmath>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : global_map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

void MapMemoryCore::initMapMemory(
  double resolution,
  int width,
  int height,
  const geometry_msgs::msg::Pose& origin,
  double inflation_radius)
{
  global_map_->info.resolution = resolution;
  global_map_->info.width = width;
  global_map_->info.height = height;
  global_map_->info.origin = origin;
  global_map_->data.assign(width * height, -1);

  inflation_radius_ = inflation_radius;

  RCLCPP_INFO(
    logger_, "Global map initialized: %.2f m/cell, %dx%d cells, inflation radius %.2f m",
    resolution, width, height, inflation_radius);
}

void MapMemoryCore::updateMap(
  const nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap,
  double robot_x,
  double robot_y,
  double robot_theta)
{
  const double local_res = local_costmap->info.resolution;
  const double local_origin_x = local_costmap->info.origin.position.x;
  const double local_origin_y = local_costmap->info.origin.position.y;
  const int local_w = static_cast<int>(local_costmap->info.width);
  const int local_h = static_cast<int>(local_costmap->info.height);

  const int map_w = static_cast<int>(global_map_->info.width);

  const double cos_t = std::cos(robot_theta);
  const double sin_t = std::sin(robot_theta);

  for (int j = 0; j < local_h; ++j) {
    for (int i = 0; i < local_w; ++i) {
      const int8_t occ_val = local_costmap->data[j * local_w + i];
      if (occ_val < 0) {

        continue;
      }

      const double lx = local_origin_x + (i + 0.5) * local_res;
      const double ly = local_origin_y + (j + 0.5) * local_res;

      const double wx = robot_x + lx * cos_t - ly * sin_t;
      const double wy = robot_y + lx * sin_t + ly * cos_t;

      int gx, gy;
      if (!worldToMapCell(wx, wy, gx, gy)) {
        continue;
      }

      int8_t& global_val = global_map_->data[gy * map_w + gx];

      if (occ_val == 0) {

        if (global_val < 0) {
          global_val = 0;
        }
      } else if (occ_val >= 100) {

        global_val = 100;
      }

    }
  }

  inflateGlobalMap();
}

void MapMemoryCore::inflateGlobalMap() {
  const int width = static_cast<int>(global_map_->info.width);
  const int height = static_cast<int>(global_map_->info.height);
  const double res = global_map_->info.resolution;
  const int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / res));

  std::vector<std::pair<int, int>> occupied_cells;
  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      if (global_map_->data[j * width + i] >= 100) {
        occupied_cells.emplace_back(i, j);
      }
    }
  }

  for (const auto& [cell_x, cell_y] : occupied_cells) {
    const int min_x = std::max(0, cell_x - inflation_cells);
    const int max_x = std::min(width - 1, cell_x + inflation_cells);
    const int min_y = std::max(0, cell_y - inflation_cells);
    const int max_y = std::min(height - 1, cell_y + inflation_cells);

    for (int y = min_y; y <= max_y; ++y) {
      for (int x = min_x; x <= max_x; ++x) {
        const double distance = std::hypot(x - cell_x, y - cell_y) * res;
        if (distance > inflation_radius_) {
          continue;
        }

        const int8_t cost = static_cast<int8_t>(100.0 * (1.0 - distance / inflation_radius_));
        if (cost <= 0) {
          continue;
        }
        int8_t& cell = global_map_->data[y * width + x];
        cell = std::max(cell, cost);
      }
    }
  }
}

bool MapMemoryCore::worldToMapCell(double wx, double wy, int& cell_x, int& cell_y) const {
  const double origin_x = global_map_->info.origin.position.x;
  const double origin_y = global_map_->info.origin.position.y;
  const double res = global_map_->info.resolution;

  cell_x = static_cast<int>(std::floor((wx - origin_x) / res));
  cell_y = static_cast<int>(std::floor((wy - origin_y) / res));

  return cell_x >= 0 &&
         cell_x < static_cast<int>(global_map_->info.width) &&
         cell_y >= 0 &&
         cell_y < static_cast<int>(global_map_->info.height);
}

nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::getMapData() const {
  return global_map_;
}

}
