#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace robot
{

class CostmapCore {
  public:

    explicit CostmapCore(const rclcpp::Logger& logger);

    void initCostmap(
      double resolution,
      int width,
      int height,
      const geometry_msgs::msg::Pose& origin,
      double inflation_radius);

    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const;

  private:

    void inflateObstacle(int cell_x, int cell_y);

    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
    rclcpp::Logger logger_;

    double inflation_radius_;
    int inflation_cells_;
};

}

#endif
