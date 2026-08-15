cmake_minimum_required(VERSION 3.8)
project(tf_throttle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(tf_throttle_node src/tf_throttle_node.cpp)
ament_target_dependencies(tf_throttle_node rclcpp tf2_msgs)

install(TARGETS tf_throttle_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()