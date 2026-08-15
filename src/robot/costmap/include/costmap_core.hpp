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
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Allocate the local grid using the parameters from params.yaml
    void initCostmap(
      double resolution,
      int width,
      int height,
      const geometry_msgs::msg::Pose& origin,
      double inflation_radius);

    // Convert a laser scan (polar coordinates in the lidar frame) into a local
    // costmap: mark obstacle hits and inflate a cost band around each of them
    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    // Retrieves the costmap data
    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmapData() const;

  private:
    // Mark every cell within the inflation radius of an obstacle with a cost
    // that falls off linearly with distance
    void inflateObstacle(int cell_x, int cell_y);

    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_data_;
    rclcpp::Logger logger_;

    double inflation_radius_;
    int inflation_cells_;
};

}  // namespace robot

#endif