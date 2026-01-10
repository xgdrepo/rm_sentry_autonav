#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

class SimpleImuToOdom {
private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Publisher odom_pub_;
    
    // 当前速度和角速度
    double vx_, vy_, wz_;
    
    // 更新时间戳
    ros::Time last_publish_time_;
    
public:
    SimpleImuToOdom() : vx_(0.0), vy_(0.0), wz_(0.0) {
        // 订阅IMU
        imu_sub_ = nh_.subscribe("/imu/data", 10, &SimpleImuToOdom::imuCallback, this);
        
        // 发布Odom
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom1", 10);
        
        last_publish_time_ = ros::Time::now();
        
        ROS_INFO("简单IMU转Odom节点启动，0.1秒刷新一次速度信息");
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 直接获取角速度
        wz_ = msg->angular_velocity.z;
        
        // 简单的速度估算（基于加速度）
        static double prev_vx = 0.0, prev_vy = 0.0;
        static ros::Time prev_time = ros::Time::now();
        
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - prev_time).toSec();
        
        if (dt > 0) {
            // 获取加速度
            double ax = msg->linear_acceleration.x;
            double ay = msg->linear_acceleration.y;
            
            // 一阶低通滤波
            double alpha = 0.2;
            vx_ = alpha * (vx_ + ax * dt) + (1 - alpha) * prev_vx;
            vy_ = alpha * (vy_ + ay * dt) + (1 - alpha) * prev_vy;
            
            // 保存当前值
            prev_vx = vx_;
            prev_vy = vy_;
            prev_time = current_time;
        }
        
        // 检查是否到达0.1秒发布间隔
        if ((current_time - last_publish_time_).toSec() >= 0.1) {
            publishOdom();
            last_publish_time_ = current_time;
        }
    }
    
    void publishOdom() {
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        // 设置速度 - 重点关注的部分
        odom_msg.twist.twist.linear.x = vx_;
        odom_msg.twist.twist.linear.y = vy_;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = wz_;
        
        // 发布消息
        odom_pub_.publish(odom_msg);
        
        ROS_DEBUG("发布: vx=%.3f, vy=%.3f, wz=%.3f", vx_, vy_, wz_);
    }
    
    void spin() {
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_imu_to_odom");
    SimpleImuToOdom node;
    node.spin();
    return 0;
}