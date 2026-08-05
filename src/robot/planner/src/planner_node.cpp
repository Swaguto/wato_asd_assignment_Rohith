#include <chrono>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&PlannerNode::timerCallback, this));
}

// TODO: implement path planning logic (A*) here
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {}

// TODO: store goal point and trigger planning
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {}

// TODO: track robot position and goal progress
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {}

// TODO: check for goal reached / timeout and replan
void PlannerNode::timerCallback() {}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
