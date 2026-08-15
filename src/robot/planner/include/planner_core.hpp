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

// ------------------- Supporting Structures -------------------

// 2D grid index
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

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // Run A* from the robot's world position to the goal on the given map and
    // store the result as a nav_msgs::Path. If the goal cell is blocked, the
    // nearest traversable cell is used instead and reported back through
    // effective_goal_x/y. Returns false if no path exists.
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
    // A* search between two map cells, with obstacle avoidance and a cost
    // penalty for cells near obstacles (inflated band)
    bool doAStar(const CellIndex &start, const CellIndex &goal, std::vector<CellIndex> &out_path);

    // Backtrack through came_from to rebuild the cell sequence of the path
    void reconstructPath(
      const std::unordered_map<CellIndex, CellIndex, CellIndexHash> &came_from,
      const CellIndex &end,
      std::vector<CellIndex> &out_path) const;

    // If the given cell is blocked or only known as an inflated band cell,
    // search outward for a better destination: first a known free cell, then
    // any traversable cell (unknown included). Returns false only if nothing
    // traversable exists nearby.
    bool relaxToFreeCell(CellIndex &cell) const;

    // BFS from 'from' for the nearest cell satisfying the given criteria.
    // require_known only accepts cells the map has actually observed.
    bool searchTraversable(const CellIndex &from, bool require_known, int max_cost, CellIndex &out) const;

    // Euclidean distance heuristic, admissible for 8-connected grids
    double heuristic(const CellIndex &a, const CellIndex &b) const;

    // Movement cost between adjacent cells: 1.0 orthogonally, sqrt(2) diagonally
    double stepCost(const CellIndex &a, const CellIndex &b) const;

    // True if the cell is inside the map and not an obstacle; also returns the
    // cell's cost (unknown cells count as traversable but expensive)
    bool isTraversable(const CellIndex &cell, int &cost) const;

    // Convert world coordinates to map indices (bounds-checked)
    bool worldToCell(double wx, double wy, CellIndex &out) const;

    // Convert map indices to world coordinates (cell center)
    void cellToWorld(const CellIndex &cell, double &wx, double &wy) const;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    nav_msgs::msg::Path::SharedPtr path_;
    rclcpp::Logger logger_;
};

}  // namespace robot

#endif