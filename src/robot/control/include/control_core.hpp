#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

// Simple pursuit controller: steer toward the next path point that lies at
// least `lookahead_distance` ahead and in front of the robot. Forward speed
// scales down as the demanded steering angle grows, is zeroed while the
// steering limit is exceeded, and is additionally cut to a crawl or zero
// when the local costmap shows a wall directly ahead.
class ControlCore {
  public:
    explicit ControlCore(const rclcpp::Logger& logger);

    void setParameters(double lookahead_distance, double steering_gain,
                       double max_steering_angle, double linear_velocity);
    void updatePath(const nav_msgs::msg::Path& path);

    geometry_msgs::msg::Twist step(double robot_x, double robot_y, double robot_yaw,
                                 const nav_msgs::msg::OccupancyGrid* local_costmap) const;

  private:
    int findLookaheadIndex(double robot_x, double robot_y, double robot_yaw) const;

    nav_msgs::msg::Path path_;
    rclcpp::Logger logger_;

    double lookahead_distance_ = 1.5;
    double steering_gain_ = 1.5;
    double max_steering_angle_ = 0.5;
    double linear_velocity_ = 1.0;
};

}

#endif
