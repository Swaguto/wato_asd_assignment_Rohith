#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

// Long-term occupancy map of the world. Each incoming local costmap (in the
// sensor frame) is reprojected into the fixed world frame using the robot
// pose, and cells are merged by keeping the highest cost ever seen there.
class MapMemoryCore {
  public:
    explicit MapMemoryCore(rclcpp::Node* node);

    // Merges one local costmap into the global map.
    void merge(const nav_msgs::msg::OccupancyGrid& local,
               double robot_x, double robot_y, double robot_yaw);

    const nav_msgs::msg::OccupancyGrid& map() const { return map_msg_; }

  private:
    void readParameters(rclcpp::Node* node);

    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid map_msg_;
    bool initialized_ = false;

    double resolution_ = 0.5;
    double origin_x_ = -15.0;
    double origin_y_ = -15.0;
    int width_ = 60;
    int height_ = 60;
};

}

#endif
