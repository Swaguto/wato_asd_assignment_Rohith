#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

bool PlannerCore::isCellFree(int row, int col, const nav_msgs::msg::OccupancyGrid& map) const
{
  if (row < 0 || row >= static_cast<int>(map.info.height) ||
      col < 0 || col >= static_cast<int>(map.info.width)) {
    return false;
  }
  return map.data[row * map.info.width + col] < 50;
}

nav_msgs::msg::Path PlannerCore::planPath(
  const nav_msgs::msg::OccupancyGrid& map,
  const geometry_msgs::msg::PointStamped& start,
  const geometry_msgs::msg::PointStamped& goal) const
{
  nav_msgs::msg::Path path;
  path.header = map.header;

  const double res = map.info.resolution;

  auto toCell = [&map, res](double x, double y) {
    int col = static_cast<int>(std::floor((x - map.info.origin.position.x) / res));
    int row = static_cast<int>(std::floor((y - map.info.origin.position.y) / res));
    return std::make_pair(row, col);
  };

  auto [sr, sc] = toCell(start.point.x, start.point.y);
  auto [gr, gc] = toCell(goal.point.x, goal.point.y);

  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  auto idx = [width](int row, int col) { return row * width + col; };

  if (!isCellFree(sr, sc, map) || !isCellFree(gr, gc, map)) {
    return path;
  }

  const int startIdx = idx(sr, sc);
  const int goalIdx = idx(gr, gc);

  std::vector<double> g_cost(width * height, std::numeric_limits<double>::infinity());
  std::vector<int> parent(width * height, -1);

  auto heuristic = [gr, gc](int row, int col) {
    return std::sqrt(static_cast<double>((row - gr) * (row - gr) + (col - gc) * (col - gc)));
  };

  using Node = std::pair<double, int>;
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

  const int dr[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dc[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  g_cost[startIdx] = 0.0;
  open.push({heuristic(sr, sc), startIdx});

  bool found = false;
  while (!open.empty()) {
    Node topNode = open.top();
    open.pop();
    const int cur = topNode.second;

    if (cur == goalIdx) {
      found = true;
      break;
    }

    const int row = cur / width;
    const int col = cur % width;

    for (int d = 0; d < 8; ++d) {
      const int nr = row + dr[d];
      const int nc = col + dc[d];
      if (!isCellFree(nr, nc, map)) {
        continue;
      }
      const int nIdx = idx(nr, nc);
      const double step = (dr[d] != 0 && dc[d] != 0) ? std::sqrt(2.0) : 1.0;
      const double cand = g_cost[cur] + step;
      if (cand < g_cost[nIdx]) {
        g_cost[nIdx] = cand;
        parent[nIdx] = cur;
        open.push({cand + heuristic(nr, nc), nIdx});
      }
    }
  }

  if (!found) {
    return path;
  }

  std::vector<std::pair<int, int>> cells;
  int cur = goalIdx;
  while (cur != startIdx) {
    cells.emplace_back(cur / width, cur % width);
    cur = parent[cur];
  }
  cells.emplace_back(sr, sc);
  std::reverse(cells.begin(), cells.end());

  for (const auto& cell : cells) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = map.info.origin.position.x + (cell.second + 0.5) * res;
    p.pose.position.y = map.info.origin.position.y + (cell.first + 0.5) * res;
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }

  return path;
}

}