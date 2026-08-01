#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp" //ros client library
#include "std_msgs/msg/string.hpp" //ros2 msgs
 
#include "costmap_core.hpp" //costmap node
 

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
    void publishMessage();
 
  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
#endif 