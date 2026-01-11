#include "ros/ros.h"  
#include "serial_com.h"
#include <sstream>  
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <string>
#include <iostream>
#include <unistd.h>  // for usleep

// 定义新的串口数据结构
#pragma pack(push, 1)  // 确保结构体字节对齐
struct Serial_Package {
    uint8_t header;         // 帧头 0xA5
    float linear_x;         // 线速度x
    float linear_y;         // 线速度y
    float angular_z;        // 角速度z
    uint8_t spin_mode;      // 小陀螺状态：0-关闭，1-开启
    uint8_t reserved[2];    // 预留字节
    uint8_t checksum;       // 校验和
};
#pragma pack(pop)

// 接收数据结构
struct Received_Package {
    uint8_t header;         // 帧头 0xA6
    uint8_t robot_hp;       // 机器人血量
    uint8_t checksum;       // 校验和
};

// 全局变量
serial::Serial sentry_ser;
std::string cmd_vel_topic;
std::atomic<bool> spin_mode_enabled(false);  // 使用原子变量确保线程安全
ros::Publisher hp_pub;           // 血量发布器
ros::Publisher spin_mode_pub;    // 小陀螺状态发布器

// 计算校验和
uint8_t calculate_checksum(const uint8_t* data, size_t length) {
    uint8_t checksum = 0;
    for(size_t i = 0; i < length; i++) {
        checksum ^= data[i];  // 使用异或校验
    }
    return checksum;
}

// 小陀螺状态回调函数
void spinModeCallback(const std_msgs::Bool::ConstPtr& msg) {
    spin_mode_enabled.store(msg->data);
    ROS_INFO("Spin mode %s", msg->data ? "enabled" : "disabled");
}

// 接收到订阅的消息后，会进入消息回调函数
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // receive the msg from cmd_vel
    ROS_INFO("Receive a /cmd_vel msg");
    ROS_INFO("The linear velocity: x=%f, y=%f, z=%f", 
              msg->linear.x, msg->linear.y, msg->linear.z);
    ROS_INFO("The angular velocity: roll=%f, pitch=%f, yaw=%f", 
              msg->angular.x, msg->angular.y, msg->angular.z);
    
    // 创建并填充串口数据包
    Serial_Package serial_package;
    serial_package.header = 0xA5;
    serial_package.linear_x = msg->linear.x;
    serial_package.linear_y = msg->linear.y;
    serial_package.angular_z = msg->angular.z;
    serial_package.spin_mode = spin_mode_enabled.load() ? 1 : 0;
    serial_package.reserved[0] = 0;
    serial_package.reserved[1] = 0;
    
    // 计算校验和（除了checksum字段本身）
    serial_package.checksum = calculate_checksum(
        reinterpret_cast<uint8_t*>(&serial_package), 
        sizeof(Serial_Package) - 1);
    
    // 发送数据
    try {
        if(sentry_ser.isOpen()) {
            sentry_ser.flush();
            size_t bytes_written = sentry_ser.write(
                reinterpret_cast<uint8_t*>(&serial_package), 
                sizeof(Serial_Package));
            
            if(bytes_written == sizeof(Serial_Package)) {
                ROS_INFO("Send data finished! Spin mode: %d", spin_mode_enabled.load());
            } else {
                ROS_WARN("Only wrote %zu bytes, expected %zu", 
                         bytes_written, sizeof(Serial_Package));
            }
        } else {
            ROS_ERROR("Serial port not open!");
        }
    } catch (serial::IOException& e) {
        ROS_ERROR("Serial write error: %s", e.what());
    }
    
    // 发布小陀螺状态
    std_msgs::Bool spin_msg;
    spin_msg.data = spin_mode_enabled.load();
    spin_mode_pub.publish(spin_msg);
}

// 读取串口数据线程函数
void readSerialThread() {
    ROS_INFO("Serial read thread started");
    
    Received_Package recv_pkg;
    uint8_t buffer[sizeof(Received_Package)];
    
    while(ros::ok()) {
        try {
            if(sentry_ser.isOpen() && sentry_ser.available()) {
                // 使用read方法读取指定长度的数据
                size_t bytes_read = sentry_ser.read(buffer, sizeof(Received_Package));
                
                if(bytes_read == sizeof(Received_Package)) {
                    // 拷贝数据到结构体
                    memcpy(&recv_pkg, buffer, sizeof(Received_Package));
                    
                    // 检查帧头
                    if(recv_pkg.header == 0xA6) {
                        // 验证校验和
                        uint8_t calculated_checksum = calculate_checksum(
                            buffer, 
                            sizeof(Received_Package) - 1);
                        
                        if(calculated_checksum == recv_pkg.checksum) {
                            // 发布血量数据
                            std_msgs::UInt8 hp_msg;
                            hp_msg.data = recv_pkg.robot_hp;
                            hp_pub.publish(hp_msg);
                            
                            ROS_INFO("Received HP data: %d", recv_pkg.robot_hp);
                        } else {
                            ROS_WARN("Checksum error for received data");
                        }
                    } else {
                        // 如果不是期望的帧头，可能需要进行帧同步
                        ROS_DEBUG("Unexpected header: 0x%02X", recv_pkg.header);
                    }
                }
            }
        } catch (serial::IOException& e) {
            ROS_ERROR("Serial read error: %s", e.what());
            // 短暂休眠后继续
            ros::Duration(0.1).sleep();
        }
        
        // 短暂休眠避免占用过多CPU
        usleep(10000);  // 10ms
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_com");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // 获取参数
    private_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel");
    std::string serial_port;
    private_nh.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    
    ROS_INFO("Using serial port: %s", serial_port.c_str());
    ROS_INFO("Subscribing to topic: %s", cmd_vel_topic.c_str());
    
    // 设置串口
    try {
        sentry_ser.setPort(serial_port);
        sentry_ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        sentry_ser.setTimeout(to);
        sentry_ser.open();
        
        // 配置串口参数（可选）
        sentry_ser.setParity(serial::parity_none);
        sentry_ser.setStopbits(serial::stopbits_one);
        sentry_ser.setBytesize(serial::eightbits);
        
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port " << serial_port << ": " << e.what());
        return -1;
    } catch (serial::SerialException& e) {
        ROS_ERROR_STREAM("Serial exception: " << e.what());
        return -1;
    } catch (...) {
        ROS_ERROR_STREAM("Unknown error opening serial port");
        return -1;
    }
    
    if(sentry_ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port " << serial_port << " opened successfully");
    } else {
        ROS_ERROR_STREAM("Failed to open serial port");
        return -1;
    }
    
    // 创建发布器
    hp_pub = nh.advertise<std_msgs::UInt8>("robot_hp", 10, true);
    spin_mode_pub = nh.advertise<std_msgs::Bool>("spin_mode_status", 10, true);
    
    // 创建订阅器
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(
        cmd_vel_topic, 10, cmdVelCallback);
    ros::Subscriber spin_sub = nh.subscribe<std_msgs::Bool>(
        "spin_mode_cmd", 10, spinModeCallback);
    
    ROS_INFO("Sentry serial node initialized");
    
    // 启动串口读取线程
    std::thread serial_read_thread(readSerialThread);
    
    // ROS主循环
    ros::MultiThreadedSpinner spinner(2);  // 使用多线程spinner
    spinner.spin();
    
    // 等待线程结束
    if(serial_read_thread.joinable()) {
        serial_read_thread.join();
    }
    
    // 关闭串口
    if(sentry_ser.isOpen()) {
        sentry_ser.close();
        ROS_INFO("Serial port closed");
    }
    
    return 0;
}