## **安装**

鱼香ros一键安装ROS：
```bash
wget http://fishros.com/install -O fishros && . fishros
安装 noetic 桌面版
```
安装需要用到的ros依赖
```bash
sudo apt-get install ros-noetic-gazebo-ros-pkgs \
ros-noetic-gazebo-ros-control \
ros-noetic-navigation \
ros-noetic-move-base \
ros-noetic-amcl \
ros-noetic-gmapping \
ros-noetic-map-server \
ros-noetic-serial \
ros-noetic-pointcloud-to-laserscan
```

## **编译步骤**

### **1. 编译Livox-SDK2**
```bash
# 假设Livox-SDK2已在ws/src目录下
cd ~/ws/src/Livox-SDK2
mkdir build
cd build
cmake .. && make -j
sudo make install
```

### **2. 编译src中的功能包**
```bash
chmod +x ~/ws/src/livox_ros_driver2/build.sh
cd ~/ws/src/livox_ros_driver2
source /opt/ros/noetic/setup.sh
./build.sh ROS1
```

## **建图流程**

### **终端1：启动建图**
```bash
cd ~/ws
source devel/setup.sh
roslaunch robot mapping.launch
```

### **终端2：启动cmd_vel控制**
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel1
```

### **终端3：建图完成，保存地图**
```bash
rosrun map_server map_saver -f map
```

## **导航流程**
### **终端1：开启导航**
```bash
roslaunch robot nav.launch
```
### **终端2：开启虚拟串口**

```bash
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```
### **修改serial_com.launch中/dev/pts/*参数**


### **终端3：输入血量**
```bash
血量 100 去增益区站点
echo -n -e "\xFA\xFB\x41\x01\x64\xF9\x12" > /dev/pts/*
```
```bash
血量 5 会补给区补血
echo -n -e "\xFA\xFB\x41\x01\x05\x85\x95" > /dev/pts/*
```
