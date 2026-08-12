#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), core_(robot::CostmapCore(this)) {
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::laserScanCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  core_.update(*msg);

  nav_msgs::msg::OccupancyGrid out = core_.buildMessage();
  // Keep the scan's stamp and frame: consumers need the sensor frame to
  // reproject these cells into world coordinates.
  out.header = msg->header;
  costmap_pub_->publish(out);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
