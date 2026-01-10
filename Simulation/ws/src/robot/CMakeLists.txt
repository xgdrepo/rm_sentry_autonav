cmake_minimum_required(VERSION 3.0.2)
project(robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_ros
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs tf2 tf2_ros tf2_sensor_msgs pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# 原来的点云变换节点
add_executable(transform_cloud_node src/transform_cloud_node.cpp)
target_link_libraries(transform_cloud_node ${catkin_LIBRARIES})

# 新的点云转激光扫描节点
add_executable(pointcloud_to_laserscan_node src/pointcloud_to_laserscan_node.cpp)
target_link_libraries(pointcloud_to_laserscan_node ${catkin_LIBRARIES})