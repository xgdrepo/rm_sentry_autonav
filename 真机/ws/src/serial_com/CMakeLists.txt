cmake_minimum_required(VERSION 3.0.2)
project(serial_com)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs serial
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 原有节点
add_executable(serial_com src/serial_com.cpp)
target_link_libraries(serial_com ${catkin_LIBRARIES})
add_dependencies(serial_com ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 添加测试节点
add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})
add_dependencies(test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})