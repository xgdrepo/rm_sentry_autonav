cmake_minimum_required(VERSION 3.0.2)
project(path_follower)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 查找Eigen3库
find_package(Eigen3 REQUIRED)

# 设置包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 设置库目录
link_directories(${catkin_LIBRARIES})

# 创建可执行文件
add_executable(path_follower_node
  src/path_follower.cpp
)

# 链接库
target_link_libraries(path_follower_node
  ${catkin_LIBRARIES}
)

# 安装目标
install(TARGETS path_follower_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# 安装配置文件（如果需要）
# install(FILES config/params.yaml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )