cmake_minimum_required(VERSION 3.8)
project(ur5_pick_and_place_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(pick_and_place_node src/pick_and_place_node.cpp)

ament_target_dependencies(pick_and_place_node
  rclcpp
  std_msgs
  trajectory_msgs
)

install(TARGETS pick_and_place_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
