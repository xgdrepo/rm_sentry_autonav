#!/bin/bash
# /home/rm/ws/start_with_shutdown_save.sh
#mkdir -p /home/rm/ws/maps/shutdown_$(date +%Y%m%d_%H%M%S)
#rosrun map_server map_saver -f /home/rm/ws/maps/shutdown_$(date +%Y%m%d_%H%M%S)


# 定义关机时保存地图
cleanup() {
    echo "Saving map on shutdown..."
    rosrun map_server map_saver -f /home/rm/ws/maps/shutdown_$(date +%Y%m%d_%H%M%S)
    exit 0
}

# 捕获退出信号
trap cleanup EXIT SIGINT SIGTERM

# 启动ROS应用
cd /home/rm/ws
source devel/setup.sh
roslaunch robot mapping_test.launch
