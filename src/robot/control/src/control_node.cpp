#include <chrono>
#include <cmath>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"),
    core_(robot::ControlCore(this->get_logger())),
    lcm_relay_() {
  this->declare_parameter("path_topic", "/path");
  this->declare_parameter("odom_topic", "/odom/filtered");
  this->declare_parameter("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter("control_period_ms", 100);
  this->declare_parameter("lookahead_distance", 1.5);
  this->declare_parameter("steering_gain", 1.5);
  this->declare_parameter("max_steering_angle", 0.5);
  this->declare_parameter("linear_velocity", 1.0);

  const std::string path_topic = this->get_parameter("path_topic").as_string();
  const std::string odom_topic = this->get_parameter("odom_topic").as_string();
  const std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
  const int period_ms = this->get_parameter("control_period_ms").as_int();
  core_.setParameters(
    this->get_parameter("lookahead_distance").as_double(),
    this->get_parameter("steering_gain").as_double(),
    this->get_parameter("max_steering_angle").as_double(),
    this->get_parameter("linear_velocity").as_double());

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    path_topic, 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&ControlNode::controlTimerCallback, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  core_.updatePath(*msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
  have_odom_ = true;
}

double ControlNode::quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void ControlNode::controlTimerCallback() {
  if (!have_odom_) {
    return;
  }

  const geometry_msgs::msg::Twist twist = core_.step(robot_x_, robot_y_, robot_yaw_);

  // Mirror the steering command onto the chassis relay so the LCM-driven
  // robot always receives fresh /cmd_vel guidance.
  lcm_relay_.setChassisOmega(twist.angular.z);

  cmd_vel_pub_->publish(twist);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
