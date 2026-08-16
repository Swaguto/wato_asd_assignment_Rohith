#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
  : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  processParameters();

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    local_costmap_topic_, 10,
    std::bind(&MapMemoryNode::localCostmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(map_pub_rate_ms_),
    std::bind(&MapMemoryNode::timerCallback, this));

  map_memory_.initMapMemory(resolution_, width_, height_, origin_, inflation_radius_);

  timerCallback();

  RCLCPP_INFO(this->get_logger(), "Map memory node ready");
}

void MapMemoryNode::processParameters() {

  this->declare_parameter<std::string>("local_costmap_topic", "/costmap");
  this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  this->declare_parameter<std::string>("map_topic", "/map");
  this->declare_parameter<int>("map_pub_rate", 1000);
  this->declare_parameter<double>("update_distance", 1.5);
  this->declare_parameter<double>("global_map.resolution", 0.5);
  this->declare_parameter<int>("global_map.width", 60);
  this->declare_parameter<int>("global_map.height", 60);
  this->declare_parameter<double>("global_map.inflation_radius", 1.5);
  this->declare_parameter<double>("global_map.origin.position.x", -15.0);
  this->declare_parameter<double>("global_map.origin.position.y", -15.0);
  this->declare_parameter<double>("global_map.origin.orientation.w", 1.0);

  local_costmap_topic_ = this->get_parameter("local_costmap_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  map_topic_ = this->get_parameter("map_topic").as_string();
  map_pub_rate_ms_ = this->get_parameter("map_pub_rate").as_int();
  update_distance_ = this->get_parameter("update_distance").as_double();
  resolution_ = this->get_parameter("global_map.resolution").as_double();
  width_ = this->get_parameter("global_map.width").as_int();
  height_ = this->get_parameter("global_map.height").as_int();
  inflation_radius_ = this->get_parameter("global_map.inflation_radius").as_double();
  origin_.position.x = this->get_parameter("global_map.origin.position.x").as_double();
  origin_.position.y = this->get_parameter("global_map.origin.position.y").as_double();
  origin_.position.z = 0.0;
  origin_.orientation.w = this->get_parameter("global_map.origin.orientation.w").as_double();
}

void MapMemoryNode::localCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  have_costmap_ = true;
  maybeFuse();
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_theta_ = quaternionToYaw(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  have_odom_ = true;
  maybeFuse();
}

void MapMemoryNode::timerCallback() {
  nav_msgs::msg::OccupancyGrid map_msg = *map_memory_.getMapData();
  map_msg.header.stamp = this->now();
  map_msg.header.frame_id = "sim_world";
  map_pub_->publish(map_msg);
}

void MapMemoryNode::maybeFuse() {
  if (!have_odom_ || !have_costmap_) {
    return;
  }

  if (have_last_fuse_) {
    const double dx = robot_x_ - last_fused_x_;
    const double dy = robot_y_ - last_fused_y_;
    if (std::hypot(dx, dy) < update_distance_) {
      return;
    }
  }

  last_fused_x_ = robot_x_;
  last_fused_y_ = robot_y_;
  have_last_fuse_ = true;

  auto costmap_copy = std::make_shared<nav_msgs::msg::OccupancyGrid>(latest_costmap_);
  map_memory_.updateMap(costmap_copy, robot_x_, robot_y_, robot_theta_);
}

double MapMemoryNode::quaternionToYaw(double x, double y, double z, double w) const {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
