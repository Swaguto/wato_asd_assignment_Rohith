#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    // Load the node's parameters from params.yaml
    void processParameters();

    // Store the newest local costmap and try to fuse it into the global map
    void localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // Track the robot's pose and try to fuse a waiting costmap
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Publish the accumulated global map at a throttled rate
    void timerCallback();

    // Fuse the latest costmap if the robot has moved far enough since the
    // last fusion (or if this is the very first observation)
    void maybeFuse();

    // Convert a quaternion into its yaw angle
    double quaternionToYaw(double x, double y, double z, double w) const;

    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string local_costmap_topic_;
    std::string odom_topic_;
    std::string map_topic_;

    int map_pub_rate_ms_;
    double update_distance_;

    double resolution_;
    int width_;
    int height_;
    double inflation_radius_;
    geometry_msgs::msg::Pose origin_;

    double robot_x_;
    double robot_y_;
    double robot_theta_;
    bool have_odom_;

    double last_fused_x_;
    double last_fused_y_;
    bool have_last_fuse_;

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool have_costmap_;
};

#endif