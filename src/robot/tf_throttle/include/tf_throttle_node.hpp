#ifndef TF_THROTTLE_NODE_HPP_
#define TF_THROTTLE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class TfThrottleNode : public rclcpp::Node {
  public:
    TfThrottleNode();

  private:
    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
    void timerCallback();

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    tf2_msgs::msg::TFMessage latest_;
    bool received_ = false;
};

#endif
