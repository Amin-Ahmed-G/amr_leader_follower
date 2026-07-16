cmake_minimum_required(VERSION 3.8)
project(amr_leader_follower)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Follower potential-field controller (leader uses Nav2, not a custom controller)
add_executable(follower_controller src/follower_controller.cpp)
ament_target_dependencies(follower_controller
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(DIRECTORY
  urdf
  config
  worlds
  launch
  dashboard
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  follower_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/ground_truth_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
