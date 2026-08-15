#include <algorithm>

#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
  : map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
    path_(std::make_shared<nav_msgs::msg::Path>()),
    logger_(logger) {}

bool PlannerCore::planPath(
  double start_x,
  double start_y,
  double goal_x,
  double goal_y,
  const nav_msgs::msg::OccupancyGrid::SharedPtr map,
  double& effective_goal_x,
  double& effective_goal_y)
{
  map_ = map;

  CellIndex start_idx;
  if (!worldToCell(start_x, start_y, start_idx)) {
    RCLCPP_WARN(logger_, "Start (%.2f, %.2f) is outside of the map. Aborting.", start_x, start_y);
    return false;
  }

  CellIndex goal_idx;
  if (!worldToCell(goal_x, goal_y, goal_idx)) {
    RCLCPP_WARN(logger_, "Goal (%.2f, %.2f) is outside of the map. Aborting.", goal_x, goal_y);
    return false;
  }

  // The goal is only considered reachable if it is known free space. If it is
  // blocked, unexplored, or inside the inflation band, drift to the nearest
  // cell that is actually clear so the robot can still get as close as safely
  // possible.
  int cost = 0;
  const bool goal_is_free = isTraversable(goal_idx, cost) && cost == 0;
  if (!goal_is_free && !relaxToFreeCell(goal_idx)) {
    RCLCPP_WARN(logger_, "No traversable cell near the goal. Aborting.");
    return false;
  }
  if (!isTraversable(start_idx, cost) && !relaxToFreeCell(start_idx)) {
    RCLCPP_WARN(logger_, "No traversable cell near the start. Aborting.");
    return false;
  }

  // Report the destination the robot should actually stop at. A goal that was
  // already known free is reported verbatim so the caller can tell a genuine
  // relaxation from mere grid quantization; otherwise report the relaxed
  // cell's center.
  if (goal_is_free) {
    effective_goal_x = goal_x;
    effective_goal_y = goal_y;
  } else {
    cellToWorld(goal_idx, effective_goal_x, effective_goal_y);
  }

  std::vector<CellIndex> path_cells;
  if (!doAStar(start_idx, goal_idx, path_cells)) {
    RCLCPP_WARN(logger_, "A* failed to find a path to the goal.");
    return false;
  }

  // Convert the cell sequence into a nav_msgs::Path in world coordinates
  path_->poses.clear();
  for (const CellIndex &cell : path_cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = map_->header;

    double wx, wy;
    cellToWorld(cell, wx, wy);

    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.orientation.w = 1.0;

    path_->poses.push_back(pose);
  }

  return true;
}

bool PlannerCore::doAStar(const CellIndex &start, const CellIndex &goal, std::vector<CellIndex> &out_path) {
  const int width = static_cast<int>(map_->info.width);
  const int height = static_cast<int>(map_->info.height);

  // Best-known cost from the start to each cell, and the predecessor used to
  // reach it (rebuilt into the final path at the end)
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  g_score[start] = 0.0;

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  open_set.push(AStarNode(start, heuristic(start, goal)));

  while (!open_set.empty()) {
    const AStarNode current = open_set.top();
    open_set.pop();

    if (current.index == goal) {
      reconstructPath(came_from, goal, out_path);
      return true;
    }

    const double current_g = g_score[current.index];

    // Expand the 8 neighbouring cells
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }

        const CellIndex neighbor(current.index.x + dx, current.index.y + dy);
        if (neighbor.x < 0 || neighbor.x >= width || neighbor.y < 0 || neighbor.y >= height) {
          continue;
        }

        int cell_cost = 0;
        if (!isTraversable(neighbor, cell_cost)) {
          continue;  // obstacle cell
        }

        // Movement cost plus a penalty that grows towards inflated cells, so
        // the path prefers staying away from obstacles
        const double tentative_g = current_g + stepCost(current.index, neighbor) + cell_cost / 10.0;

        const auto it = g_score.find(neighbor);
        if (it != g_score.end() && tentative_g >= it->second) {
          continue;  // already reached this cell more cheaply
        }

        g_score[neighbor] = tentative_g;
        came_from[neighbor] = current.index;
        open_set.push(AStarNode(neighbor, tentative_g + heuristic(neighbor, goal)));
      }
    }
  }

  return false;  // open set exhausted, no path exists
}

void PlannerCore::reconstructPath(
  const std::unordered_map<CellIndex, CellIndex, CellIndexHash> &came_from,
  const CellIndex &end,
  std::vector<CellIndex> &out_path) const
{
  out_path.clear();
  CellIndex current = end;
  out_path.push_back(current);

  auto it = came_from.find(current);
  while (it != came_from.end()) {
    current = it->second;
    out_path.push_back(current);
    it = came_from.find(current);
  }

  std::reverse(out_path.begin(), out_path.end());
}

bool PlannerCore::relaxToFreeCell(CellIndex &cell) const {
  // Destinations must be cells the robot has actually seen as clear (cost 0).
  // Banded cells within the inflation radius are reachable for driving but too
  // close for the robot's body to stop at, and unknown cells may be obstacle
  // interiors that no path can reach.
  return searchTraversable(cell, true, 0, cell);
}

bool PlannerCore::searchTraversable(
  const CellIndex &from,
  bool require_known,
  int max_cost,
  CellIndex &out) const
{
  const int width = static_cast<int>(map_->info.width);
  const int height = static_cast<int>(map_->info.height);
  constexpr int kMaxSearchRadius = 12;

  std::vector<std::vector<bool>> visited(width, std::vector<bool>(height, false));
  std::queue<CellIndex> queue;
  queue.push(from);
  visited[from.x][from.y] = true;

  while (!queue.empty()) {
    const CellIndex current = queue.front();
    queue.pop();

    int cost = 0;
    if (isTraversable(current, cost) && (!require_known || cost <= max_cost)) {
      out = current;
      return true;
    }

    // Stop expanding once we've searched far enough away from the original cell
    if (std::max(std::abs(current.x - from.x), std::abs(current.y - from.y)) >= kMaxSearchRadius) {
      continue;
    }

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }

        const int nx = current.x + dx;
        const int ny = current.y + dy;
        if (nx < 0 || nx >= width || ny < 0 || ny >= height || visited[nx][ny]) {
          continue;
        }

        visited[nx][ny] = true;
        queue.push(CellIndex(nx, ny));
      }
    }
  }

  return false;
}

double PlannerCore::heuristic(const CellIndex &a, const CellIndex &b) const {
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  return std::sqrt(dx * dx + dy * dy);
}

double PlannerCore::stepCost(const CellIndex &a, const CellIndex &b) const {
  const int dx = std::abs(a.x - b.x);
  const int dy = std::abs(a.y - b.y);
  return (dx + dy == 2) ? std::sqrt(2.0) : 1.0;
}

bool PlannerCore::isTraversable(const CellIndex &cell, int &cost) const {
  const int width = static_cast<int>(map_->info.width);
  const int height = static_cast<int>(map_->info.height);

  if (cell.x < 0 || cell.x >= width || cell.y < 0 || cell.y >= height) {
    cost = 127;
    return false;
  }

  const int8_t val = map_->data[cell.y * width + cell.x];
  if (val < 0) {
    // Unknown cells are traversable but expensive: A* prefers known paths
    cost = 100;
    return true;
  }

  // Free cells and thin inflation band are traversable; the rest of the band
  // (cells within ~1.125 m of an obstacle with the 1.5 m inflation radius) is
  // off-limits. The thin band passes the corridor cells between the inflation
  // rings of two obstacles, which are genuinely clear of the obstacles.
  cost = static_cast<int>(val);
  return cost <= 25;
}

bool PlannerCore::worldToCell(double wx, double wy, CellIndex &out) const {
  const double origin_x = map_->info.origin.position.x;
  const double origin_y = map_->info.origin.position.y;
  const double res = map_->info.resolution;

  out.x = static_cast<int>(std::floor((wx - origin_x) / res));
  out.y = static_cast<int>(std::floor((wy - origin_y) / res));

  return out.x >= 0 &&
         out.x < static_cast<int>(map_->info.width) &&
         out.y >= 0 &&
         out.y < static_cast<int>(map_->info.height);
}

void PlannerCore::cellToWorld(const CellIndex &cell, double &wx, double &wy) const {
  const double origin_x = map_->info.origin.position.x;
  const double origin_y = map_->info.origin.position.y;
  const double res = map_->info.resolution;

  wx = origin_x + (cell.x + 0.5) * res;
  wy = origin_y + (cell.y + 0.5) * res;
}

nav_msgs::msg::Path::SharedPtr PlannerCore::getPath() const {
  return path_;
}

}  // namespace robot