#include <chrono>
#include <cmath>

#include "lcm_relay.hpp"

namespace robot
{

namespace
{
constexpr double kOmegaEpsilon = 1e-3;
}

LcmRelay::LcmRelay() : Node("lcm_relay")
{
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&LcmRelay::timerCallback, this));
}

void LcmRelay::setChassisOmega(double omega) {
  chassis_omega_ = omega;
}

void LcmRelay::timerCallback() {
  if (std::abs(chassis_omega_) <= kOmegaEpsilon) {
    return;
  }

  geometry_msgs::msg::Twist twist;
  twist.angular.z = chassis_omega_;
  cmd_vel_pub_->publish(twist);
  chassis_omega_ = 0.0;
}

}