#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:

    void processParameters();

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    void poseGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void timerCallback();

    void setGoal(double x, double y);

    void planAndPublish();

    void cancelGoal();

    enum class State { WAITING_FOR_GOAL, TRACKING_GOAL };
    State state_;

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string map_topic_;
    std::string goal_topic_;
    std::string odom_topic_;
    std::string path_topic_;

    double goal_tolerance_;
    double plan_timeout_;

    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    std::mutex map_mutex_;

    double goal_x_;
    double goal_y_;

    double effective_goal_x_;
    double effective_goal_y_;

    rclcpp::Time plan_start_time_;

    bool have_odom_;
    double odom_x_;
    double odom_y_;
};

#endif
