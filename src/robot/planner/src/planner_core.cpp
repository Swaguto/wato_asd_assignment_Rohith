#include <algorithm>
#include <cstddef>

#include "planner_core.hpp"

namespace robot
{

namespace
{
// Cells with occupancy above this are considered physically blocked. With
// the 0.5 m global map resolution the gradient only reaches this level on
// the obstacle itself, so most avoidance is handled by the cost penalty.
constexpr int8_t kBlockedCost = 90;
// Cost added to a path step per unit of occupancy in the destination cell.
// A cell in the inflated gradient (e.g. cost 40) adds ~4 m of equivalent
// travel, which keeps the path on the outer gradient ring so the body
// clears boxes and walls.
constexpr double kCostPenaltyScale = 0.10;
// Grid coordinates used by the search queue.
using Cell = PlannerCore::Cell;
struct QueueEntry {
  double f;
  Cell cell;
};
struct EntryOrder {
  bool operator()(const QueueEntry& a, const QueueEntry& b) const {
    return a.f > b.f;
  }
};
}

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

bool PlannerCore::plan(const nav_msgs::msg::OccupancyGrid& map,
                       double start_x, double start_y,
                       double goal_x, double goal_y,
                       std::vector<geometry_msgs::msg::PoseStamped>& out) const {
  out.clear();

  Cell start;
  Cell goal;
  if (!toCell(map, start_x, start_y, start)) {
    RCLCPP_WARN(logger_, "Start (%.2f, %.2f) is outside the map.", start_x, start_y);
    return false;
  }
  if (!toCell(map, goal_x, goal_y, goal)) {
    RCLCPP_WARN(logger_, "Goal (%.2f, %.2f) is outside the map.", goal_x, goal_y);
    return false;
  }

  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const size_t cell_count = static_cast<size_t>(width) * height;

  const auto index = [width](const Cell& c) {
    return static_cast<size_t>(c.y) * width + c.x;
  };

  std::vector<double> g(cell_count, std::numeric_limits<double>::infinity());
  std::vector<Cell> parent(cell_count, Cell{-1, -1});

  std::priority_queue<QueueEntry, std::vector<QueueEntry>, EntryOrder> open;
  g[index(start)] = 0.0;
  open.push({heuristic(start, goal), start});

  bool reached = false;
  while (!open.empty()) {
    const Cell current = open.top().cell;
    open.pop();

    if (current.x == goal.x && current.y == goal.y) {
      reached = true;
      break;
    }

    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const Cell next{current.x + dx, current.y + dy};
        if (next.x < 0 || next.x >= width || next.y < 0 || next.y >= height) {
          continue;
        }
        const int8_t cost = cellCost(map, next);
        if (cost > kBlockedCost) {
          continue;
        }
        const double travel = stepCost(current, next) + static_cast<double>(cost) * kCostPenaltyScale;
        const double candidate = g[index(current)] + travel;
        const size_t nidx = index(next);
        if (candidate < g[nidx]) {
          g[nidx] = candidate;
          parent[nidx] = current;
          open.push({candidate + heuristic(next, goal), next});
        }
      }
    }
  }

  if (!reached) {
    RCLCPP_WARN(logger_, "A* could not find a route from (%d, %d) to (%d, %d).",
                start.x, start.y, goal.x, goal.y);
    return false;
  }

  // Walk the parent chain back to the start and flip it around.
  std::vector<Cell> cells;
  Cell c = goal;
  while (c.x >= 0) {
    cells.push_back(c);
    if (c.x == start.x && c.y == start.y) {
      break;
    }
    c = parent[index(c)];
  }
  std::reverse(cells.begin(), cells.end());

  const double res = map.info.resolution;
  for (const Cell& cell : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = map.info.origin.position.x + (static_cast<double>(cell.x) + 0.5) * res;
    pose.pose.position.y = map.info.origin.position.y + (static_cast<double>(cell.y) + 0.5) * res;
    pose.pose.orientation.w = 1.0;
    out.push_back(pose);
  }
  return true;
}

bool PlannerCore::toCell(const nav_msgs::msg::OccupancyGrid& map,
                         double wx, double wy, Cell& cell) const {
  const double res = map.info.resolution;
  cell.x = static_cast<int>(std::floor((wx - map.info.origin.position.x) / res));
  cell.y = static_cast<int>(std::floor((wy - map.info.origin.position.y) / res));
  if (cell.x < 0 || cell.x >= static_cast<int>(map.info.width) ||
      cell.y < 0 || cell.y >= static_cast<int>(map.info.height)) {
    return false;
  }
  return true;
}

int8_t PlannerCore::cellCost(const nav_msgs::msg::OccupancyGrid& map, const Cell& cell) {
  return map.data[static_cast<size_t>(cell.y) * map.info.width + cell.x];
}

double PlannerCore::heuristic(const Cell& a, const Cell& b) {
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

double PlannerCore::stepCost(const Cell& from, const Cell& to) {
  const int dx = std::abs(from.x - to.x);
  const int dy = std::abs(from.y - to.y);
  return (dx == 1 && dy == 1) ? std::sqrt(2.0) : 1.0;
}

}
