#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:

    void processParameters();

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void controlLoop();

    double quaternionToYaw(double x, double y, double z, double w) const;

    robot::ControlCore control_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string path_topic_;
    std::string odom_topic_;
    std::string cmd_vel_topic_;

    int control_period_ms_;
    double lookahead_distance_;
    double max_steering_angle_;
    double steering_gain_;
    double linear_velocity_;
    double goal_tolerance_;

    double robot_x_;
    double robot_y_;
    double robot_theta_;
    bool have_odom_;
};

#endif
