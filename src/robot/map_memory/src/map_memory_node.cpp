#include <chrono>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
  );
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
  );
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
  map_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&MapMemoryNode::updateMap, this)
  );
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;      // copy: `msg` is a pointer, `*msg` dereferences
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  latest_odom_ = *msg;
  odom_received_ = true;
}

void MapMemoryNode::updateMap(){
  if (!costmap_received_ || !odom_received_){
    return;
  }
  auto map = latest_costmap_;    // start from latest local costmap
  // TODO: use latest_odom_ (x, y, yaw from msg->pose.pose) to
  //       transform/merge the local costmap into the global map frame
  map.header.frame_id = "map";
  map_pub_->publish(map);        // publish the merged grid
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
