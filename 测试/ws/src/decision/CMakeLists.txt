cmake_minimum_required(VERSION 3.0.2)
project(decision)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
)

# 系统依赖
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs tf
)

# 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}

)

# 可执行文件
add_executable(decision_node src/decision_node.cpp)

# 链接库
target_link_libraries(decision_node
  ${catkin_LIBRARIES}

)

# 安装
install(TARGETS decision_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

