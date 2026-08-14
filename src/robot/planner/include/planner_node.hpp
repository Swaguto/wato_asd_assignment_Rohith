#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "planner_core.hpp"

// Goal manager + replanner. While a goal is active, every fresh /map
// message triggers a re-plan from the current odometry; the goal expires
// when reached or when the planning budget runs out.
class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void statusTimerCallback();

    void attemptPlan();
    void clearGoal();

    robot::PlannerCore core_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    nav_msgs::msg::OccupancyGrid latest_map_;
    bool have_map_ = false;
    double odom_x_ = 0.0;
    double odom_y_ = 0.0;
    bool have_odom_ = false;

    bool goal_active_ = false;
    geometry_msgs::msg::PointStamped goal_;
    rclcpp::Time goal_started_;

    double goal_tolerance_ = 1.5;
    double plan_timeout_s_ = 60.0;
};

#endif
