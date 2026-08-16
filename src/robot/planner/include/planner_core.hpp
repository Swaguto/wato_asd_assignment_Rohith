#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {

    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

struct AStarNode
{
  CellIndex index;
  double f_score;

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {

    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    bool planPath(
      double start_x,
      double start_y,
      double goal_x,
      double goal_y,
      const nav_msgs::msg::OccupancyGrid::SharedPtr map,
      double& effective_goal_x,
      double& effective_goal_y);

    nav_msgs::msg::Path::SharedPtr getPath() const;

  private:

    bool doAStar(const CellIndex &start, const CellIndex &goal, std::vector<CellIndex> &out_path);

    void reconstructPath(
      const std::unordered_map<CellIndex, CellIndex, CellIndexHash> &came_from,
      const CellIndex &end,
      std::vector<CellIndex> &out_path) const;

    bool relaxToFreeCell(CellIndex &cell) const;

    bool searchTraversable(const CellIndex &from, bool require_known, int max_cost, CellIndex &out) const;

    double heuristic(const CellIndex &a, const CellIndex &b) const;

    double stepCost(const CellIndex &a, const CellIndex &b) const;

    bool isTraversable(const CellIndex &cell, int &cost) const;

    bool worldToCell(double wx, double wy, CellIndex &out) const;

    void cellToWorld(const CellIndex &cell, double &wx, double &wy) const;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::Path::SharedPtr path_;
    rclcpp::Logger logger_;
};

}

#endif
