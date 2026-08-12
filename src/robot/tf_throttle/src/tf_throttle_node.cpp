#include <chrono>

#include "tf_throttle_node.hpp"

TfThrottleNode::TfThrottleNode() : Node("tf_throttle")
{
  sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf_raw", 10,
    std::bind(&TfThrottleNode::tfCallback, this, std::placeholders::_1));
  pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TfThrottleNode::timerCallback, this));
}

void TfThrottleNode::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
  latest_ = *msg;
  received_ = true;
}

void TfThrottleNode::timerCallback() {
  if (!received_) {
    return;
  }
  pub_->publish(latest_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfThrottleNode>());
  rclcpp::shutdown();
  return 0;
}
