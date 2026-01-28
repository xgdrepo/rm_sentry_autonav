cmake_minimum_required(VERSION 3.0.2)
project(imu_to_odom)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(

  LIBRARIES imu_to_odom
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 主版本（完整功能）
add_executable(imu_to_odom_node src/imu_to_odom_node.cpp)
target_link_libraries(imu_to_odom_node ${catkin_LIBRARIES})

