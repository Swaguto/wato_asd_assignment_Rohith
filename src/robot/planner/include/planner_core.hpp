#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    nav_msgs::msg::Path planPath(const nav_msgs::msg::OccupancyGrid& map,
                                 const geometry_msgs::msg::PointStamped& start,
                                 const geometry_msgs::msg::PointStamped& goal) const;

  private:
    bool isCellFree(int row, int col, const nav_msgs::msg::OccupancyGrid& map) const;
    rclcpp::Logger logger_;
};

}

#endif