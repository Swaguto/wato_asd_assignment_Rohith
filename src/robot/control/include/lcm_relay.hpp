#ifndef LCM_RELAY_HPP_
#define LCM_RELAY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

// The Husarion chassis is driven over LCM and will not act on a stale
// /cmd_vel message. This relay mirrors the control loop's angular command
// on /cmd_vel; a missing beat (no turn requested) is published as a
// zero-turn message so the chassis always receives fresh guidance.
class LcmRelay : public rclcpp::Node {
  public:
    LcmRelay();

    void setChassisOmega(double omega);

  private:
    void timerCallback();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double chassis_omega_ = 0.0;
};

}

#endif
