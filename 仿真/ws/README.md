# **项目部署指南**

## **依赖安装**
```bash
sudo apt-get install ros-noetic-gazebo-ros-pkgs \
ros-noetic-gazebo-ros-control \
ros-noetic-navigation \
ros-noetic-move-base \
ros-noetic-amcl \
ros-noetic-gmapping \
ros-noetic-map-server
```
鱼香ros一键安装必要的ROS包：
```bash
通过终端输入：wget http://fishros.com/install -O fishros && . fishros
选择并安装：rosdepc
然后一键安装需要用的库
cd ~/ws
rosdepc update
rosdepc install --from-paths src --ignore-src -r -y
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

### **2. 编译livox_ros_driver2**
```bash
# 将livox_laser_simulation从src移动到ws根目录
mv ~/ws/src/livox_laser_simulation ~/ws/

# 编译驱动
cd ~/ws/src/livox_ros_driver2
source /opt/ros/noetic/setup.sh
./build.sh ROS1
```

### **3. 编译livox_laser_simulation**
```bash
# 将livox_ros_driver2从src移动到ws根目录,再把livox_laser_simulation移动回来
mv ~/ws/src/livox_ros_driver2 ~/ws/
mv ~/ws/livox_laser_simulation ~/ws/src/

# 编译整个工作空间
cd ~/ws
source /opt/ros/noetic/setup.sh
catkin_make
```

## **建图流程**

### **终端1：启动Gazebo仿真**
```bash
cd ~/ws
source devel/setup.sh
roslaunch robot gazebo.launch
```

### **终端2：启动建图**
```bash
cd ~/ws
source devel/setup.sh
roslaunch robot mapping.launch
```

### **终端3：启动控制**
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### **终端4：启动rviz,并订阅map话题实时显示建图过程**
```bash
rviz
```
### **终端5：建图完成，保存地图**
```bash
rosrun map_server map_saver
```

## **导航流程**

### **单终端启动导航**
```bash
cd ~/ws
source devel/setup.sh
roslaunch robot nav.launch
```

## **注意事项**


### **文件权限问题**
如果遇到权限问题，运行：
```bash
chmod +x ~/ws/src/livox_ros_driver2/build.sh
```
