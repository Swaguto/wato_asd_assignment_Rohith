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

    // Allocate the global map using the parameters from params.yaml
    void initMapMemory(
      double resolution,
      int width,
      int height,
      const geometry_msgs::msg::Pose& origin,
      double inflation_radius);

    // Fuse a local (lidar-frame) costmap into the world-frame global map using
    // the robot's current pose. New observations override older ones; cells the
    // local costmap has not seen are left untouched.
    void updateMap(
      const nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap,
      double robot_x,
      double robot_y,
      double robot_theta);

    // Retrieves the accumulated global map
    nav_msgs::msg::OccupancyGrid::SharedPtr getMapData() const;

  private:
    // Convert world coordinates into a global map cell index (bounds-checked)
    bool worldToMapCell(double wx, double wy, int& cell_x, int& cell_y) const;

    // Redraw the inflation band around every occupied cell directly on the
    // global grid, so band values reflect true distances to obstacles instead
    // of smeared local-costmap values
    void inflateGlobalMap();

    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
    double inflation_radius_ = 0.0;
    rclcpp::Logger logger_;
};

}  // namespace robot

#endif