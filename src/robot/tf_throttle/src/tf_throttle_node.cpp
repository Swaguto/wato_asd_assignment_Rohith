#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace robot
{

class TfThrottleNode : public rclcpp::Node
{
public:
  TfThrottleNode()
  : Node("tf_throttle")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_raw", qos,
      [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) { latest_ = msg; });
    pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", qos);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      [this]() {
        if (latest_) {
          pub_->publish(*latest_);
        }
      });
  }

private:
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_;
  tf2_msgs::msg::TFMessage::SharedPtr latest_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robot::TfThrottleNode>());
  rclcpp::shutdown();
  return 0;
}