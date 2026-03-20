#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8.h>  // 改为UInt8
#include <thread>
#include <atomic>

// 全局变量声明
extern serial::Serial sentry_ser;
extern std::string cmd_vel_topic;
extern std::atomic<uint8_t> spin_mode_enabled;  // 改为uint8_t
extern ros::Publisher hp_pub;
extern ros::Publisher spin_mode_pub;

// 函数声明
uint8_t calculate_checksum(const uint8_t* data, size_t length);
void spinModeCallback(const std_msgs::UInt8::ConstPtr& msg);  // 改为UInt8
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
void readSerialThread();

#endif // SERIAL_COM_H