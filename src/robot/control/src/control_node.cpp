#include <chrono>
#include <cmath>
#include <memory>

#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger())) {
  processParameters();

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    path_topic_, 10,
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10,
    std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(control_period_ms_),
    std::bind(&ControlNode::controlLoop, this));

  control_.initControlCore(
    lookahead_distance_, max_steering_angle_, steering_gain_,
    linear_velocity_, goal_tolerance_);

  RCLCPP_INFO(this->get_logger(), "Control node ready");
}

void ControlNode::processParameters() {
  // Declare all ROS2 parameters (defaults match config/params.yaml)
  this->declare_parameter<std::string>("path_topic", "/path");
  this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  this->declare_parameter<int>("control_period_ms", 100);
  this->declare_parameter<double>("lookahead_distance", 1.5);
  this->declare_parameter<double>("max_steering_angle", 0.5);
  this->declare_parameter<double>("steering_gain", 1.5);
  this->declare_parameter<double>("linear_velocity", 1.0);
  this->declare_parameter<double>("goal_tolerance", 1.0);

  // Retrieve parameters and store them in member variables
  path_topic_ = this->get_parameter("path_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
  control_period_ms_ = this->get_parameter("control_period_ms").as_int();
  lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
  max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
  steering_gain_ = this->get_parameter("steering_gain").as_double();
  linear_velocity_ = this->get_parameter("linear_velocity").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  control_.updatePath(*msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_theta_ = quaternionToYaw(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  have_odom_ = true;
}

void ControlNode::controlLoop() {
  if (!have_odom_) {
    return;  // can't steer without knowing where we are
  }

  geometry_msgs::msg::Twist cmd;
  if (control_.isPathEmpty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No active path; standing by.");
  } else if (control_.isGoalReached(robot_x_, robot_y_)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Goal reached; stopping.");
  } else {
    cmd = control_.calculateControlCommand(robot_x_, robot_y_, robot_theta_);
  }

  cmd_vel_pub_->publish(cmd);
}

double ControlNode::quaternionToYaw(double x, double y, double z, double w) const {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}