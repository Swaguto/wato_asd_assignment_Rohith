#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// Builds a fixed grid occupancy map from the lidar scan. The grid lives in
// the sensor frame (the robot is always at the grid centre), so no robot
// pose is needed here: map_memory later reprojects the cells into world
// coordinates using the odometry.
class CostmapCore {
  public:
    explicit CostmapCore(rclcpp::Node* node);

    // Processes one laser scan: clears the grid, stamps obstacle hits and
    // spreads an inflated gradient around each hit.
    void update(const sensor_msgs::msg::LaserScan& scan);

    // Assembles the current grid into an OccupancyGrid message.
    nav_msgs::msg::OccupancyGrid buildMessage() const;

  private:
    void readParameters(rclcpp::Node* node);
    void inflate();

    rclcpp::Logger logger_;
    std::vector<int8_t> cells_;

    double resolution_ = 0.4;         // [m/cell]
    double inflation_radius_ = 1.5;   // [m]
    double origin_x_ = -24.0;         // [m] grid bottom-left corner (sensor frame)
    double origin_y_ = -24.0;
    int width_ = 120;
    int height_ = 120;
    bool configured_ = false;
};

}

#endif
