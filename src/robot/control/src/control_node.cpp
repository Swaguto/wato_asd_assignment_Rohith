#include <chrono>
#include <cmath>

#include "control_node.hpp"

namespace
{
constexpr double kLookAhead = 0.3;
}

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&ControlNode::timerCallback, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  path_ = msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_ = msg;
}

void ControlNode::timerCallback() {
  if (!path_ || !odom_ || path_->poses.empty()) {
    return;
  }

  const double robot_x = odom_->pose.pose.position.x;
  const double robot_y = odom_->pose.pose.position.y;

  geometry_msgs::msg::PoseStamped target = path_->poses.back();
  for (const auto& waypoint : path_->poses) {
    const double dx = waypoint.pose.position.x - robot_x;
    const double dy = waypoint.pose.position.y - robot_y;
    if (std::hypot(dx, dy) >= kLookAhead) {
      target = waypoint;
      break;
    }
  }

  cmd_vel_pub_->publish(control_.computeTwist(target, *odom_));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}