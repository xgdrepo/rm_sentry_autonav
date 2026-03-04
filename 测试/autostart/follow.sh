#!/bin/bash
sleep 2
cd /home/rm/ws
source devel/setup.sh
sudo chmod 666 /dev/ttyACM0
roslaunch serial_com serial_test.launch