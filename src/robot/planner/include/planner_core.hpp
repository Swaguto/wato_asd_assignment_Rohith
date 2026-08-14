#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

// Grid-based A* planner. Travel cost is the path length plus a penalty
// proportional to the occupancy value of each cell, so the search prefers
// routes that stay away from inflated obstacle gradients.
class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    struct Cell {
      int x;
      int y;
    };

    // Plans from (start_x, start_y) to (goal_x, goal_y) on the given map.
    // On success the cell centres are appended to `out` and true is
    // returned; on failure `out` is left empty and false is returned.
    bool plan(const nav_msgs::msg::OccupancyGrid& map,
              double start_x, double start_y,
              double goal_x, double goal_y,
              std::vector<geometry_msgs::msg::PoseStamped>& out) const;

  private:
    bool toCell(const nav_msgs::msg::OccupancyGrid& map,
                double wx, double wy, Cell& cell) const;
    static int8_t cellCost(const nav_msgs::msg::OccupancyGrid& map, const Cell& cell);
    bool relaxCell(const nav_msgs::msg::OccupancyGrid& map,
                   Cell& cell, const char* label) const;
    static double heuristic(const Cell& a, const Cell& b);
    static double stepCost(const Cell& from, const Cell& to);
    void lineOfSightShortcut(const nav_msgs::msg::OccupancyGrid& map, std::vector<Cell>& cells) const;
    bool hasLineOfSight(const nav_msgs::msg::OccupancyGrid& map, const Cell& from, const Cell& to) const;

    rclcpp::Logger logger_;
};

}

#endif
