#!/bin/bash
# /home/rm/ws/start_with_shutdown_save.sh

# 启动ROS应用
cd /home/rm/ws
source devel/setup.sh

# 启动ROS应用（在后台运行）
roslaunch robot mapping_test.launch &
ROS_PID=$!

# 启动监控脚本（在新终端中）
gnome-terminal -- bash -c "
  echo 'Monitoring ROS process (PID: $ROS_PID) for shutdown...'
  # 等待ROS进程结束
  wait $ROS_PID
  # ROS进程结束后，保存地图
  cd /home/rm/ws
  source devel/setup.sh
  echo 'Saving map on shutdown...'
  rosrun map_server map_saver -f /home/rm/ws/maps/shutdown_\$(date +%Y%m%d_%H%M%S)
  echo 'Map saved. Press Enter to close...'
  read
"

# 等待ROS进程
wait $ROS_PID
