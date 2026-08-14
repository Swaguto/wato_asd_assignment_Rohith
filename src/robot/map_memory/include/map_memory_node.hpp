#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishTimerCallback();
    static double quaternionToYaw(const geometry_msgs::msg::Quaternion& q);

    robot::MapMemoryCore core_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool have_costmap_ = false;
    bool have_odom_ = false;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    double update_distance_ = 1.5;
    double update_time_ = 15.0;
    rclcpp::Time last_merge_time_;
    double last_merge_x_ = 0.0;
    double last_merge_y_ = 0.0;
    bool ever_merged_ = false;
};

#endif
