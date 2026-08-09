#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class MapMemoryCore {
  
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    nav_msgs::msg::OccupancyGrid updateMap(const nav_msgs::msg::OccupancyGrid& costmap,
                                           const nav_msgs::msg::Odometry& odom);

    void intialize(const nav_msgs::msg::OccupancyGrid& sample);

      void mergeCostmap(const nav_msgs::msg::OccupancyGrid& local, const NavPose& pose);

    const nav_msgs::msg::OccupancyGrid& getMap() const {
      return global_map_;
    }

    intialize(sample):
      global_map_ = sample;                                                                                                                                                                                 
      width = sample.info.width;
      global_map_.header.frame_id = "map";

    }                                         

    //   for each cell in local costmap: 
    //   if cell is occupied:
    //     transform cell to global frame using robot_x, robot_y, robot_yaw
    //     update global_map_ at transformed cell position


    updateCostmap(local, pose):
    robot_x = odom.pose.pose.position.x;
    robot_yaw = tf2::getYaw(odom.pose.pose.orientation);
    for (int i = 0; i < local.data.size(); ++i) {
      if (local.data[i] < 50) {                                     
        continue;
      }
      int local_row = i / local.info.width;
      int local_col = i % local.info.width;
      double local_x = local.info.origin.position.x + (local_col + 0.5) * local.info.resolution;
      double local_y = local.info.origin.position.y + (local_row + 0.5) * local.info.resolution;


      double global_x = robot_x + (local_x * std::cos(robot_yaw) - local_y * std::sin(robot_yaw));
      double global_y = robot_y + (local_x * std::sin(robot_yaw) + local_y * std::cos(robot_yaw));

      int global_col = static_cast<int>(std::floor((global_x - global_map_.info.origin.position.x) / global_map_.info.resolution));
      int global_row = static_cast<int>(std::floor((global_y - global_map_.info.origin.position.y) / global_map_.info.resolution));


      if (global_row < 0 || global_row >= static_cast<int>(global_map_.info.height) ||
          global_col < 0 || global_col >= static_cast<int>(global_map_.info.width)) {
        continue;
      }

      int global_idx = global_row * global_map_.info.width + global_col;
      global_map_.data[global_idx] = std::max(global_map_.data[global_idx], local.data[i]);


    }
   
    if (!costmap_received_ || !odom_received_){
      return;                           
    }                 
   
    if (global_map_.data.empty()) {
      RCLCPP_WARN(logger_, "Global map is empty. Cannot update.");
      return;
    }

    if (global_map.intialized == false){
      core.initMap(latest_costmap_)
    }

    core.updateCostmap(latest_costmap_, latest_odom_);
    auto map = core.getMap(); 
    map.header.stamp = rclcpp::Clock().now();
    map_pub_->publish(map)





  private:
    rclcpp::Logger logger_;
};

}  

#endif  
