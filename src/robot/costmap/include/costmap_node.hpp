#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:

    CostmapNode();

  private:

    void processParameters();

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    robot::CostmapCore costmap_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    std::string laserscan_topic_;
    std::string costmap_topic_;

    double resolution_;
    int width_;
    int height_;
    geometry_msgs::msg::Pose origin_;
    double inflation_radius_;
};

#endif
