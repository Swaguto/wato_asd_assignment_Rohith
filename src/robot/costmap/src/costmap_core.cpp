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
  node->declare_parameter("costmap.inflation_radius", 2.0);

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
  const int infl_cells =
    std::max(1, static_cast<int>(std::ceil(inflation_radius_ / resolution_)));
  const int cost_step = std::max(1, kOccupiedCost / infl_cells);
  const int diag_step =
    std::max(1, static_cast<int>(std::ceil(cost_step * std::sqrt(2.0))));

  // Seed the gradient from every obstacle cell, then relax it outwards.
  std::vector<int8_t> ring(cells_.size(), 0);
  for (size_t i = 0; i < cells_.size(); ++i) {
    if (cells_[i] >= kSeedCost) {
      ring[i] = kOccupiedCost;
    }
  }

  for (int d = 0; d < infl_cells; ++d) {
    propagateGradient(ring, cost_step, diag_step);
  }

  for (size_t i = 0; i < cells_.size(); ++i) {
    if (ring[i] > cells_[i]) {
      cells_[i] = ring[i];
    }
  }
}

void CostmapCore::propagateGradient(std::vector<int8_t>& ring,
                                    int step, int diag_step) const {
  // One full forward+backward pass moves the gradient one ring further in
  // all eight directions. Both straight (step) and diagonal (diag_step)
  // neighbours are relaxed so the inflated zone is circular, not diamond.
  const auto relax = [this, &ring, step, diag_step](int y, int x) {
    const size_t idx = static_cast<size_t>(y) * width_ + x;
    int best = ring[idx];

    auto consider = [&best, &ring](size_t n, int s) {
      best = std::max(best, static_cast<int>(ring[n]) - s);
    };
    if (x > 0) {
      consider(idx - 1, step);
    }
    if (x < width_ - 1) {
      consider(idx + 1, step);
    }
    if (y > 0) {
      consider(idx - static_cast<size_t>(width_), step);
    }
    if (y < height_ - 1) {
      consider(idx + static_cast<size_t>(width_), step);
    }
    if (x > 0 && y > 0) {
      consider(idx - static_cast<size_t>(width_) - 1, diag_step);
    }
    if (x < width_ - 1 && y > 0) {
      consider(idx - static_cast<size_t>(width_) + 1, diag_step);
    }
    if (x > 0 && y < height_ - 1) {
      consider(idx + static_cast<size_t>(width_) - 1, diag_step);
    }
    if (x < width_ - 1 && y < height_ - 1) {
      consider(idx + static_cast<size_t>(width_) + 1, diag_step);
    }

    if (best < 0) {
      best = 0;
    }
    ring[idx] = static_cast<int8_t>(best);
  };

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      relax(y, x);
    }
  }
  for (int y = height_ - 1; y >= 0; --y) {
    for (int x = width_ - 1; x >= 0; --x) {
      relax(y, x);
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
