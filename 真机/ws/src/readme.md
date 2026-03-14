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
rosrun tf2_tools view_frames.py