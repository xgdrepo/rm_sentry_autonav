cmake_minimum_required(VERSION 3.0.2)
project(dynamic_map_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamic_map_generator_node src/dynamic_map_generator_node.cpp)
target_link_libraries(dynamic_map_generator_node ${catkin_LIBRARIES})