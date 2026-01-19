cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pure_pursuit_controller src/pure_pursuit.cpp)
target_link_libraries(pure_pursuit_controller ${catkin_LIBRARIES})

install(TARGETS pure_pursuit_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)