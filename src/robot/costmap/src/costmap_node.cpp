#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  processParameters();

  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    laserscan_topic_, 10,
    std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(costmap_topic_, 10);

  costmap_.initCostmap(resolution_, width_, height_, origin_, inflation_radius_);

  RCLCPP_INFO(this->get_logger(), "Costmap node ready");
}

void CostmapNode::processParameters() {

  this->declare_parameter<std::string>("laserscan_topic", "/lidar");
  this->declare_parameter<std::string>("costmap_topic", "/costmap");
  this->declare_parameter<double>("costmap.resolution", 0.4);
  this->declare_parameter<int>("costmap.width", 120);
  this->declare_parameter<int>("costmap.height", 120);
  this->declare_parameter<double>("costmap.origin.position.x", -24.0);
  this->declare_parameter<double>("costmap.origin.position.y", -24.0);
  this->declare_parameter<double>("costmap.origin.orientation.w", 1.0);
  this->declare_parameter<double>("costmap.inflation_radius", 1.5);

  laserscan_topic_ = this->get_parameter("laserscan_topic").as_string();
  costmap_topic_ = this->get_parameter("costmap_topic").as_string();
  resolution_ = this->get_parameter("costmap.resolution").as_double();
  width_ = this->get_parameter("costmap.width").as_int();
  height_ = this->get_parameter("costmap.height").as_int();
  origin_.position.x = this->get_parameter("costmap.origin.position.x").as_double();
  origin_.position.y = this->get_parameter("costmap.origin.position.y").as_double();
  origin_.position.z = 0.0;
  origin_.orientation.w = this->get_parameter("costmap.origin.orientation.w").as_double();
  inflation_radius_ = this->get_parameter("costmap.inflation_radius").as_double();
}

void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.updateCostmap(msg);

  nav_msgs::msg::OccupancyGrid costmap_msg = *costmap_.getCostmapData();
  costmap_msg.header = msg->header;
  costmap_pub_->publish(costmap_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
