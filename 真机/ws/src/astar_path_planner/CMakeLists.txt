cmake_minimum_required(VERSION 3.0.2)
project(astar_path_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(astar_planner_node src/astar_planner_node.cpp)
target_link_libraries(astar_planner_node ${catkin_LIBRARIES})
add_dependencies(astar_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})