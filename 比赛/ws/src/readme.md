编译
cd ~/ws/src/livox_ros_driver2
source /opt/ros/noetic/setup.sh
./build.sh ROS1

开启导航
source devel/setup.sh
roslaunch robot nav.launch

开启虚拟串口
socat -d -d pty,raw,echo=0 pty,raw,echo=0

修改serial_com.launch中/dev/pts/*参数

血量85
echo -n -e "\xFA\xFB\x41\x02\x55\x00" > /dev/pts/5
血量395
echo -n -e "\xFA\xFB\x41\x02\x8B\x01" > /dev/pts/5


killall gzserver gzclient


# 安装依赖（如未安装）
sudo apt-get install ros-noetic-tf2-tools

# 查看当前TF树并保存为PDF
sudo chmod 666 /dev/ttyACM0
rosrun tf2_tools view_frames.py

rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel1

rosbag record -O v.bag /cmd_vel1
rosbag play v.bag

rosrun map_server map_saver -f map


rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 2, y: 0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'

真实建图 导航 定位 路径跟随01

1.串口号要改成USB
2.红蓝方都要测试路线
3.血量
4.小陀螺
5.高度
