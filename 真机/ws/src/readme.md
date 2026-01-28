修改MID360_config.json里的lidar_configs的ip：192.168.1.121（雷达最后两位sn码是21），pcl_data_type：2
msg_MID360.launch里的xfer_format：2

编译
cd ~/ws/src/livox_ros_driver2
source /opt/ros/noetic/setup.sh
./build.sh ROS1

开启导航
cd ~/ws
source devel/setup.sh
roslaunch robot nav.launch
