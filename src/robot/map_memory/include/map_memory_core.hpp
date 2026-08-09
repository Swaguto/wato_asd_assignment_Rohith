#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void updateMap(const nav_msgs::msg::OccupancyGrid& costmap,
                   const nav_msgs::msg::Odometry& odom);

    const nav_msgs::msg::OccupancyGrid& getMap() const;

  private:
    void initialize(const nav_msgs::msg::OccupancyGrid& sample);
    void mergeCostmap(const nav_msgs::msg::OccupancyGrid& local,
                      const nav_msgs::msg::Odometry& odom);

    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;
    bool initialized_ = false;
};

}

#endif