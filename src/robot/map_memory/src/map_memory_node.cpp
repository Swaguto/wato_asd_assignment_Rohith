#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
: Node("map_memory"), core_(robot::MapMemoryCore(this)) {
  this->declare_parameter("local_costmap_topic", "/costmap");
  this->declare_parameter("odom_topic", "/odom/filtered");
  this->declare_parameter("map_topic", "/map");
  this->declare_parameter("map_pub_rate", 3000);
  this->declare_parameter("update_distance", 1.5);
  this->declare_parameter("update_time", 15.0);

  const std::string costmap_topic = this->get_parameter("local_costmap_topic").as_string();
  const std::string odom_topic = this->get_parameter("odom_topic").as_string();
  const std::string map_topic = this->get_parameter("map_topic").as_string();
  const int map_pub_rate = this->get_parameter("map_pub_rate").as_int();
  update_distance_ = this->get_parameter("update_distance").as_double();
  update_time_ = this->get_parameter("update_time").as_double();

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic, 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, 10);
  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(map_pub_rate),
    std::bind(&MapMemoryNode::publishTimerCallback, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = *msg;
  have_costmap_ = true;

  if (!have_odom_) {
    return;
  }
  // Like the reference implementation, an all-zero costmap (e.g. a dead
  // lidar) carries no information: skip the merge so existing memory is not
  // replaced by it.
  const bool empty_costmap =
    std::all_of(latest_costmap_.data.begin(), latest_costmap_.data.end(),
                [](int8_t v) { return v == 0; });
  if (empty_costmap) {
    return;
  }
  // Merge whenever the robot has moved far enough from the last merge point:
  // obstacles the lidar has just seen must reach the planner's map before
  // the robot outruns the old plan. The timer below still forces a merge
  // while idling.
  const bool first = !ever_merged_;
  const bool travelled =
    std::hypot(robot_x_ - last_merge_x_, robot_y_ - last_merge_y_) >= update_distance_;
  if (first || travelled) {
    core_.merge(latest_costmap_, robot_x_, robot_y_, robot_yaw_);
    last_merge_x_ = robot_x_;
    last_merge_y_ = robot_y_;
    last_merge_time_ = this->now();
    ever_merged_ = true;
  }
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_yaw_ = quaternionToYaw(msg->pose.pose.orientation);
  have_odom_ = true;
}

double MapMemoryNode::quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void MapMemoryNode::publishTimerCallback() {
  if (!have_costmap_ || !have_odom_) {
    return;
  }

  const rclcpp::Time now = this->now();

  // While the robot idles the travel gate above never trips; force a merge
  // every update_time seconds so the map cannot grow stale beside a wall.
  if (ever_merged_ && (now - last_merge_time_).seconds() >= update_time_) {
    core_.merge(latest_costmap_, robot_x_, robot_y_, robot_yaw_);
    last_merge_x_ = robot_x_;
    last_merge_y_ = robot_y_;
    last_merge_time_ = now;
  }

  nav_msgs::msg::OccupancyGrid out = core_.map();
  out.header.stamp = now;
  map_pub_->publish(out);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
