#include <chrono>
#include <cmath>
#include <memory>

#include "planner_node.hpp"

namespace
{
constexpr double kGoalReachedDist = 0.2;
}

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_map_ = *msg;
  map_received_ = true;
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  latest_goal_ = *msg;
  goal_received_ = true;
  goal_reached_ = false;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  latest_odom_ = *msg;
  odom_received_ = true;
}

void PlannerNode::timerCallback() {
  if (!goal_reached_ && map_received_ && goal_received_ && odom_received_) {
    planPath();
  }
}

void PlannerNode::planPath() {
  if (!map_received_ || !goal_received_ || !odom_received_) {
    return;
  }

  const double dx = latest_goal_.point.x - latest_odom_.pose.pose.position.x;
  const double dy = latest_goal_.point.y - latest_odom_.pose.pose.position.y;
  if (std::hypot(dx, dy) < kGoalReachedDist) {
    goal_reached_ = true;
    return;
  }

  geometry_msgs::msg::PointStamped start;
  start.header.frame_id = latest_map_.header.frame_id;
  start.point.x = latest_odom_.pose.pose.position.x;
  start.point.y = latest_odom_.pose.pose.position.y;
  start.point.z = 0.0;

  nav_msgs::msg::Path path = planner_.planPath(latest_map_, start, latest_goal_);
  path.header.frame_id = latest_map_.header.frame_id;
  path.header.stamp = this->now();

  if (path.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "Could not find a path");
    return;
  }
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}