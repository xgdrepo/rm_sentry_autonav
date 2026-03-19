#!/bin/bash

readonly VERSION_ROS1="ROS1"
readonly VERSION_ROS2="ROS2"
readonly VERSION_HUMBLE="humble"

pushd `pwd` > /dev/null
cd `dirname $0`
echo "Working Path: "`pwd`

ROS_VERSION=""
ROS_HUMBLE=""

# Set working ROS version
if [ "$1" = "ROS2" ]; then
    ROS_VERSION=${VERSION_ROS2}
elif [ "$1" = "humble" ]; then
    ROS_VERSION=${VERSION_ROS2}
    ROS_HUMBLE=${VERSION_HUMBLE}
elif [ "$1" = "ROS1" ]; then
    ROS_VERSION=${VERSION_ROS1}
else
    echo "Invalid Argument"
    exit
fi
echo "ROS version is: "$ROS_VERSION

# exit

# substitute the files/folders: CMakeList.txt, package.xml(s)
if [ ${ROS_VERSION} = ${VERSION_ROS1} ]; then
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f package_ROS1.xml package.xml
elif [ ${ROS_VERSION} = ${VERSION_ROS2} ]; then
    if [ -f package.xml ]; then
        rm package.xml
    fi
    cp -f package_ROS2.xml package.xml
    cp -rf launch_ROS2/ launch/
fi

# build
pushd `pwd` > /dev/null
if [ $ROS_VERSION = ${VERSION_ROS1} ]; then
    cd ../../
    
    # ========== 修改开始：分步编译解决依赖问题 ==========
    echo "步骤 1/3: 生成 livox_ros_driver2 的消息文件..."
    catkin_make -DROS_EDITION=${VERSION_ROS1} livox_ros_driver2_generate_messages
    
    echo "步骤 2/3: 编译 livox_ros_driver2 驱动节点..."
    catkin_make -DROS_EDITION=${VERSION_ROS1} livox_ros_driver2_node
    
    echo "步骤 3/3: 编译所有包..."
    catkin_make -DROS_EDITION=${VERSION_ROS1} -j4
    
    # 或者使用下面这个更全面的方法：
    # echo "步骤 1/4: 生成所有包的消息文件..."
    # catkin_make -DROS_EDITION=${VERSION_ROS1} generate_messages
    
    # echo "步骤 2/4: 单独编译 livox_ros_driver2..."
    # catkin_make -DROS_EDITION=${VERSION_ROS1} --only-pkg-with-deps livox_ros_driver2
    
    # echo "步骤 3/4: 编译其他包..."
    # catkin_make -DROS_EDITION=${VERSION_ROS1} --start-with livox_ros_driver2 --continue-on-failure -j4
    
    # ========== 修改结束 ==========
    
elif [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    cd ../../
    colcon build --cmake-args -DROS_EDITION=${VERSION_ROS2} -DHUMBLE_ROS=${ROS_HUMBLE}
fi
popd > /dev/null

# remove the substituted folders/files
if [ $ROS_VERSION = ${VERSION_ROS2} ]; then
    rm -rf launch/
fi

popd > /dev/null
