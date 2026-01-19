cmake_minimum_required(VERSION 3.0.2)
project(smooth_path_follower)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf
  actionlib
  actionlib_msgs
  dynamic_reconfigure
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(smooth_path_follower src/smooth_path_follower.cpp)
target_link_libraries(smooth_path_follower ${catkin_LIBRARIES})

install(TARGETS smooth_path_follower
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)