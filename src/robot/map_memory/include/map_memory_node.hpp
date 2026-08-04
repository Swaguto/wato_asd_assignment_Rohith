#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"


class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateMap();

  private:
    robot::MapMemoryCore map_memory_;

    // 2 subscribers + 1 publisher + 1 timer
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_; //sub to occupancy grid
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; //sub to odometry filtered
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_; //pub to map topic
    rclcpp::TimerBase::SharedPtr map_timer_;

    // storage: the latest data each subscriber received
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    nav_msgs::msg::Odometry latest_odom_;
    bool costmap_received_ = false;      // so updateMap knows data exists
    bool odom_received_ = false;
};

#endif
