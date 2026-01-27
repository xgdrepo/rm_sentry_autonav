编译
cd ~/ws/src/livox_ros_driver2
source /opt/ros/noetic/setup.sh
./build.sh ROS1

开启导航
roslaunch robot nav.launch

开启虚拟串口
socat -d -d pty,raw,echo=0 pty,raw,echo=0

修改serial_com.launch中/dev/pts/*参数

血量100
echo -n -e "\xFA\xFB\x41\x01\x64\xF9\x12" > /dev/pts/*
血量5
echo -n -e "\xFA\xFB\x41\x01\x05\x85\x95" > /dev/pts/*
