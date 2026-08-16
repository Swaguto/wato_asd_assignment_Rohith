#include <chrono>
#include <cmath>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode()
  : Node("planner"), state_(State::WAITING_FOR_GOAL), planner_(robot::PlannerCore(this->get_logger())) {
  processParameters();

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, 10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    goal_topic_, 10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  pose_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    std::bind(&PlannerNode::poseGoalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10,
    std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&PlannerNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Planner node ready");
}

void PlannerNode::processParameters() {

  this->declare_parameter<std::string>("map_topic", "/map");
  this->declare_parameter<std::string>("goal_topic", "/goal_point");
  this->declare_parameter<std::string>("odom_topic", "/odom/filtered");
  this->declare_parameter<std::string>("path_topic", "/path");
  this->declare_parameter<double>("goal_tolerance", 1.0);
  this->declare_parameter<double>("plan_timeout_seconds", 60.0);

  map_topic_ = this->get_parameter("map_topic").as_string();
  goal_topic_ = this->get_parameter("goal_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  path_topic_ = this->get_parameter("path_topic").as_string();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  plan_timeout_ = this->get_parameter("plan_timeout_seconds").as_double();
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_ = msg;
  }

  if (state_ == State::TRACKING_GOAL) {
    planAndPublish();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received goal: (%.2f, %.2f)", msg->point.x, msg->point.y);
  setGoal(msg->point.x, msg->point.y);
}

void PlannerNode::poseGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received pose goal: (%.2f, %.2f)",
              msg->pose.position.x, msg->pose.position.y);
  setGoal(msg->pose.position.x, msg->pose.position.y);
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_x_ = msg->pose.pose.position.x;
  odom_y_ = msg->pose.pose.position.y;
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::TRACKING_GOAL) {
    return;
  }

  const double elapsed = (now() - plan_start_time_).seconds();

  if (elapsed > plan_timeout_) {
    RCLCPP_WARN(this->get_logger(), "Goal timed out after %.1f s; cancelling.", elapsed);
    cancelGoal();
    return;
  }

  if (!have_odom_) {
    return;
  }

  const double dx = odom_x_ - effective_goal_x_;
  const double dy = odom_y_ - effective_goal_y_;
  if (std::hypot(dx, dy) < goal_tolerance_) {
    RCLCPP_INFO(this->get_logger(), "Goal reached in %.1f s.", elapsed);
    cancelGoal();
  }
}

void PlannerNode::setGoal(double x, double y) {
  if (!map_) {
    RCLCPP_WARN(this->get_logger(), "No map received yet; ignoring goal.");
    return;
  }

  goal_x_ = x;
  goal_y_ = y;
  state_ = State::TRACKING_GOAL;
  plan_start_time_ = now();

  planAndPublish();
}

void PlannerNode::planAndPublish() {
  if (!have_odom_) {
    RCLCPP_WARN(this->get_logger(), "No odometry received yet; cannot plan.");
    return;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr map;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_;
  }
  if (!map) {
    return;
  }

  if (!planner_.planPath(odom_x_, odom_y_, goal_x_, goal_y_, map,
                         effective_goal_x_, effective_goal_y_)) {
    RCLCPP_WARN(this->get_logger(), "Planning failed; abandoning this goal.");
    cancelGoal();
    return;
  }

  if (std::hypot(effective_goal_x_ - goal_x_, effective_goal_y_ - goal_y_) > 0.05) {
    RCLCPP_WARN(this->get_logger(), "Goal (%.2f, %.2f) is blocked; driving to nearest free cell (%.2f, %.2f).",
                goal_x_, goal_y_, effective_goal_x_, effective_goal_y_);
  }

  nav_msgs::msg::Path path_msg = *planner_.getPath();
  path_msg.header.stamp = now();
  path_msg.header.frame_id = map->header.frame_id;

  path_pub_->publish(path_msg);
}

void PlannerNode::cancelGoal() {
  state_ = State::WAITING_FOR_GOAL;

  nav_msgs::msg::Path empty_path;
  empty_path.header.stamp = now();
  path_pub_->publish(empty_path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
