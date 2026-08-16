#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void initMapMemory(
      double resolution,
      int width,
      int height,
      const geometry_msgs::msg::Pose& origin,
      double inflation_radius);

    void updateMap(
      const nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap,
      double robot_x,
      double robot_y,
      double robot_theta);

    nav_msgs::msg::OccupancyGrid::SharedPtr getMapData() const;

  private:

    bool worldToMapCell(double wx, double wy, int& cell_x, int& cell_y) const;

    void inflateGlobalMap();

    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
    double inflation_radius_ = 0.0;
    rclcpp::Logger logger_;
};

}

#endif
