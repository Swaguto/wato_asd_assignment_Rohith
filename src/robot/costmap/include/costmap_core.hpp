#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void setPose(double x, double y, double yaw);
    nav_msgs::msg::OccupancyGrid getCostmapMsg();

  private:
    rclcpp::Logger logger_;

    std::vector<int8_t> grid_;
    double resolution_ = 0.1;
    int width_ = 100;
    int height_ = 100;
    double origin_x_ = -5.0;
    double origin_y_ = -5.0;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;
    double inflation_radius_ = 1.0;
    const int8_t MAX_COST_ = 100; 

    void initializeCostmap();
    void markObstacles(const sensor_msgs::msg::LaserScan& scan);
    void inflateObstacles();
    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);
};

}
#endif