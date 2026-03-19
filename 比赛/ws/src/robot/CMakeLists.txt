cmake_minimum_required(VERSION 3.0.2)
project(robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs

)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)