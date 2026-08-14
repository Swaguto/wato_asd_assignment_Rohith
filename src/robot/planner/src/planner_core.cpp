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
// Cost added to a path step per unit of occupancy in the destination cell,
// matching the reference tuning: a cost-40 gradient cell adds ~1.6 m of
// equivalent travel, so inflated zones are avoided without forcing detours.
constexpr double kCostPenaltyScale = 0.04;
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
  if (!relaxCell(map, start, "start")) {
    RCLCPP_WARN(logger_, "No traversable cell within 1 m of the start. Aborting.");
    return false;
  }
  if (!relaxCell(map, goal, "goal")) {
    RCLCPP_WARN(logger_, "No traversable cell within 1 m of the goal. Aborting.");
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

  lineOfSightShortcut(map, cells);

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

bool PlannerCore::relaxCell(const nav_msgs::msg::OccupancyGrid& map,
                            Cell& cell, const char* label) const {
  // A wall face or its inflated ring can cover the requested cell even for a
  // perfectly valid goal: goals are expected to sit on free floor with the
  // robot stopping within goal_tolerance of them. When the cell is blocked,
  // snap to the nearest traversable neighbour (up to two cells away, which
  // is still inside the 1.5 m goal tolerance at 0.5 m resolution).
  if (cellCost(map, cell) <= kBlockedCost) {
    return true;
  }

  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const int start_cost = cellCost(map, cell);

  for (int radius = 1; radius <= 2; ++radius) {
    Cell best{-1, -1};
    int best_cost = std::numeric_limits<int>::max();
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (dx * dx + dy * dy > radius * radius) {
          continue;
        }
        const Cell candidate{cell.x + dx, cell.y + dy};
        if (candidate.x < 0 || candidate.x >= width ||
            candidate.y < 0 || candidate.y >= height) {
          continue;
        }
        const int cost = cellCost(map, candidate);
        if (cost > kBlockedCost || cost >= best_cost) {
          continue;
        }
        best = candidate;
        best_cost = cost;
      }
    }
    if (best.x >= 0) {
      RCLCPP_WARN(logger_, "%s cell (%d, %d) blocked (cost %d); relaxed to (%d, %d).",
                  label, cell.x, cell.y, start_cost, best.x, best.y);
      cell = best;
      return true;
    }
  }
  return false;
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

void PlannerCore::lineOfSightShortcut(const nav_msgs::msg::OccupancyGrid& map,
                                      std::vector<Cell>& cells) const {
  // Replaces the stair-step A* cell chain with a polyline that skips every
  // intermediate cell whose centre has a clear line of sight to the next
  // waypoint: the pursued path becomes long smooth diagonals.
  if (cells.size() <= 2) {
    return;
  }

  std::vector<Cell> out;
  out.push_back(cells.front());
  size_t i = 0;
  while (i < cells.size() - 1) {
    size_t j = cells.size() - 1;
    while (j > i + 1 && !hasLineOfSight(map, cells[i], cells[j])) {
      --j;
    }
    out.push_back(cells[j]);
    i = j;
  }
  cells.swap(out);
}

bool PlannerCore::hasLineOfSight(const nav_msgs::msg::OccupancyGrid& map,
                                 const Cell& from, const Cell& to) const {
  // Bresenham between cell centres; every cell crossed must be traversable.
  int x0 = from.x, y0 = from.y;
  const int x1 = to.x, y1 = to.y;
  const int dx = std::abs(x1 - x0), dy = -std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  while (true) {
    if (cellCost(map, Cell{x0, y0}) > kBlockedCost) {
      return false;
    }
    if (x0 == x1 && y0 == y1) {
      return true;
    }
const int e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x0 += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y0 += sy;
      }
      // The crossing cells may be traversable yet sit deep in an inflated
      // ring: the robot pushes through such corridors while scraping the
      // wall. Only shortcut when the band stays clear of half-inflation.
      if (cellCost(map, Cell{x0, y0}) >= 50) {
        return false;
      }
  }
}

}
