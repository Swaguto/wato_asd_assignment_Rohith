#include <chrono>
#include <cmath>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode()
: Node("planner"), core_(robot::PlannerCore(this->get_logger())) {
  this->declare_parameter("map_topic", "/map");
  this->declare_parameter("goal_topic", "/goal_point");
  this->declare_parameter("odom_topic", "/odom/filtered");
  this->declare_parameter("path_topic", "/path");
  this->declare_parameter("goal_tolerance", 1.5);
  this->declare_parameter("plan_timeout_seconds", 60.0);

  const std::string map_topic = this->get_parameter("map_topic").as_string();
  const std::string goal_topic = this->get_parameter("goal_topic").as_string();
  const std::string odom_topic = this->get_parameter("odom_topic").as_string();
  const std::string path_topic = this->get_parameter("path_topic").as_string();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  plan_timeout_s_ = this->get_parameter("plan_timeout_seconds").as_double();

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic, 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    goal_topic, 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
  status_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::statusTimerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_map_ = *msg;
  have_map_ = true;
  if (goal_active_) {
    attemptPlan();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  if (goal_active_) {
    RCLCPP_WARN(this->get_logger(), "A goal is already active; ignoring (%.2f, %.2f).",
                msg->point.x, msg->point.y);
    return;
  }
  if (!have_map_) {
    RCLCPP_WARN(this->get_logger(), "No map yet; cannot plan to (%.2f, %.2f).",
                msg->point.x, msg->point.y);
    return;
  }
  goal_ = *msg;
  goal_active_ = true;
  goal_started_ = this->now();
  RCLCPP_INFO(this->get_logger(), "New goal: (%.2f, %.2f).", msg->point.x, msg->point.y);
  attemptPlan();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_x_ = msg->pose.pose.position.x;
  odom_y_ = msg->pose.pose.position.y;
  have_odom_ = true;
}

void PlannerNode::statusTimerCallback() {
  if (!goal_active_) {
    return;
  }
  const double elapsed = (this->now() - goal_started_).seconds();
  if (elapsed > plan_timeout_s_) {
    RCLCPP_WARN(this->get_logger(), "Goal timed out after %.1f s; clearing it.", elapsed);
    clearGoal();
    return;
  }
  if (have_odom_) {
    const double dist = std::hypot(odom_x_ - goal_.point.x, odom_y_ - goal_.point.y);
    if (dist < goal_tolerance_) {
      RCLCPP_INFO(this->get_logger(), "Goal reached (%.2f m away); clearing it.", dist);
      clearGoal();
      return;
    }
  }
  // Refresh the path at 2 Hz even when the map has not changed: the robot
  // outruns the old plan between map updates and would overshoot corners.
  if (have_map_ && have_odom_) {
    attemptPlan();
  }
}

void PlannerNode::attemptPlan() {
  if (!have_odom_) {
    RCLCPP_WARN(this->get_logger(), "No odometry yet; clearing goal.");
    clearGoal();
    return;
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = latest_map_.header.frame_id;
  path.header.stamp = this->now();

  if (!core_.plan(latest_map_, odom_x_, odom_y_, goal_.point.x, goal_.point.y, path.poses)) {
    // Layouts and discovered walls change while driving: a plan that is
    // impossible now may become possible after the next map merge. Keep the
    // goal active and retry on the next status tick instead of giving up.
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Planning failed; retrying for current goal.");
    return;
  }

  path_pub_->publish(path);
}

void PlannerNode::clearGoal() {
  goal_active_ = false;

  // Tell the controller to stop: an empty path.
  nav_msgs::msg::Path stop_path;
  stop_path.header.frame_id = have_map_ ? latest_map_.header.frame_id : "sim_world";
  stop_path.header.stamp = this->now();
  path_pub_->publish(stop_path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
