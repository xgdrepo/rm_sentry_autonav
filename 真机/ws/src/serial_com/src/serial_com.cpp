#include "ros/ros.h"
#include "serial_com.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <string>
#include <iostream>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <iomanip>
#include <chrono>

// 原来的数据帧结构体
#pragma pack(push, 1)
struct FrameHeader {
    uint8_t start1 = 0xFA;
    uint8_t start2 = 0xFB;
    uint8_t cmd_id;
    uint8_t data_len;
};

struct RadarSendFrame {
    FrameHeader header;
    float linear_x;
    float linear_y;
    float angular_z;
    uint8_t spin_mode;
};

struct RadarRecvFrame {
    FrameHeader header;
    uint16_t hp;
};
#pragma pack(pop)

// 全局变量
serial::Serial sentry_ser;
std::string cmd_vel_topic;
std::atomic<uint8_t> spin_mode_enabled(0);
ros::Publisher hp_pub;
ros::Publisher spin_mode_pub;

// CDC设备连接函数
bool connectCDCDevice() {
    ROS_INFO("Connecting to CDC device...");
    
    try {
        if (sentry_ser.isOpen()) {
            ROS_INFO("CDC device connected successfully");
            
            // 等待设备稳定
            ros::Duration(0.5).sleep();
            
            // 清空缓冲区
            sentry_ser.flush();
            
            return true;
        } else {
            ROS_ERROR("Serial port not open");
            return false;
        }
    } catch (serial::IOException &e) {
        ROS_ERROR("Connection failed: %s", e.what());
        return false;
    }
}

// 小陀螺状态回调函数
void spinModeCallback(const std_msgs::UInt8::ConstPtr &msg) {
    uint8_t new_spin_mode = msg->data;
    spin_mode_enabled.store(new_spin_mode);
    
    std_msgs::UInt8 status_msg;
    status_msg.data = new_spin_mode;
    spin_mode_pub.publish(status_msg);
}

// 发送函数
void sendCmdVel(const geometry_msgs::Twist::ConstPtr &msg) {
    uint8_t current_spin_mode = spin_mode_enabled.load();
    
    RadarSendFrame frame;
    frame.header.start1 = 0xFA;
    frame.header.start2 = 0xFB;
    frame.header.cmd_id = 0x21;
    frame.header.data_len = sizeof(frame.linear_x) + sizeof(frame.linear_y) + 
                           sizeof(frame.angular_z) + sizeof(frame.spin_mode);
    
    frame.linear_x = std::round(msg->linear.x * 1000.0f) / 1000.0f;
    frame.linear_y = std::round(msg->linear.y * 1000.0f) / 1000.0f;
    frame.angular_z = std::round(msg->angular.z * 1000.0f) / 1000.0f;
    frame.spin_mode = current_spin_mode;
    
    try {
        if (sentry_ser.isOpen()) {
            const uint8_t* data = reinterpret_cast<const uint8_t*>(&frame);
            size_t bytes_written = sentry_ser.write(data, sizeof(RadarSendFrame));
            
            if (bytes_written != sizeof(RadarSendFrame)) {
                ROS_WARN_THROTTLE(1.0, "[TX] Partial write: %zu/%zu bytes", 
                        bytes_written, sizeof(RadarSendFrame));
            }
        }
    } catch (serial::IOException &e) {
        ROS_ERROR("Serial write error: %s", e.what());
    }
}

// 串口读取线程 - 优化版本（不使用getFd）
void readSerialThread() {
    ROS_INFO("Serial read thread started - optimized for low latency");
    uint8_t buffer[256];
    size_t buffer_index = 0;
    uint8_t temp_buffer[256];
    
    // 统计变量，用于调试
    int idle_count = 0;
    
    while (ros::ok()) {
        try {
            if (sentry_ser.isOpen()) {
                // 使用非阻塞方式检查数据可用性
                size_t available = sentry_ser.available();
                
                if (available > 0) {
                    // 有数据可用，立即读取
                    size_t bytes_read = sentry_ser.read(temp_buffer, 
                                                       std::min(available, sizeof(temp_buffer)));
                    
                    if (bytes_read > 0) {
                        // 重置空闲计数
                        idle_count = 0;
                        
                        // 可选：打印接收时间戳用于调试（降低频率）
                        static auto last_print_time = std::chrono::steady_clock::now();
                        auto now = std::chrono::steady_clock::now();
                        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
                                     (now - last_print_time).count();
                        
                        if (elapsed > 1000) { // 每秒打印一次
                            ROS_INFO("[RX] Active - received %zu bytes", bytes_read);
                            last_print_time = now;
                        }
                        
                        // 处理接收到的数据
                        for (size_t i = 0; i < bytes_read; i++) {
                            buffer[buffer_index++] = temp_buffer[i];
                            
                            // 尝试解析帧
                            if (buffer_index >= sizeof(RadarRecvFrame)) {
                                // 检查帧头
                                if (buffer[0] == 0xFA && buffer[1] == 0xFB) {
                                    RadarRecvFrame recv_frame;
                                    memcpy(&recv_frame, buffer, sizeof(RadarRecvFrame));
                                    
                                    if (recv_frame.header.cmd_id == 0x41) {
                                        std_msgs::UInt16 hp_msg;
                                        hp_msg.data = recv_frame.hp;
                                        hp_pub.publish(hp_msg);
                                        ROS_INFO("[RX] HP received: %d", recv_frame.hp);
                                        
                                        // 清除已处理的数据
                                        buffer_index = 0;
                                    } else {
                                        // 命令ID不匹配，移动缓冲区
                                        if (buffer_index > 1) {
                                            memmove(buffer, buffer + 1, buffer_index - 1);
                                            buffer_index--;
                                        } else {
                                            buffer_index = 0;
                                        }
                                    }
                                } else {
                                    // 帧头不匹配，移动缓冲区
                                    if (buffer_index > 1) {
                                        memmove(buffer, buffer + 1, buffer_index - 1);
                                        buffer_index--;
                                    } else {
                                        buffer_index = 0;
                                    }
                                }
                            }
                            
                            // 防止缓冲区溢出
                            if (buffer_index >= sizeof(buffer)) {
                                ROS_WARN_THROTTLE(1.0, "Buffer overflow, resetting");
                                buffer_index = 0;
                            }
                        }
                    }
                    
                    // 数据读取后短暂休眠，避免CPU占用过高
                    usleep(500);  // 0.5ms
                    
                } else {
                    // 没有数据时，稍微休眠以减少CPU占用
                    idle_count++;
                    
                    // 动态调整休眠时间
                    if (idle_count > 100) {
                        // 长时间空闲，可以休眠稍长一点
                        usleep(2000);  // 2ms
                    } else {
                        usleep(500);   // 0.5ms - 保持响应性
                    }
                }
            } else {
                // 串口未打开，等待时间稍长
                usleep(10000);  // 10ms
            }
            
        } catch (serial::IOException &e) {
            ROS_ERROR_THROTTLE(1.0, "Serial read error: %s", e.what());
            ros::Duration(0.5).sleep();
        } catch (const std::exception& e) {
            ROS_ERROR_THROTTLE(1.0, "Unexpected error: %s", e.what());
            ros::Duration(0.5).sleep();
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "serial_com");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // 设置日志级别
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, 
        ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }
    
    // 获取参数
    private_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel1");
    std::string serial_port;
    private_nh.param<std::string>("serial_port", serial_port, "/dev/ttyACM0");
    
    ROS_INFO("Using serial port: %s", serial_port.c_str());
    ROS_INFO("Subscribing to topic: %s", cmd_vel_topic.c_str());
    
    // 设置串口
    try {
        sentry_ser.setPort(serial_port);
        sentry_ser.setBaudrate(115200);
        
        // 优化超时设置 - 更短的超时时间以提高响应性
        serial::Timeout to = serial::Timeout(
            10,      // inter_byte_timeout (10ms)
            50,      // read_timeout_constant (50ms)
            0,       // read_timeout_multiplier
            1000,    // write_timeout_constant
            100      // write_timeout_multiplier
        );
        sentry_ser.setTimeout(to);
        
        // 打开串口
        sentry_ser.open();
        
        // 设置串口参数
        sentry_ser.setParity(serial::parity_none);
        sentry_ser.setStopbits(serial::stopbits_one);
        sentry_ser.setBytesize(serial::eightbits);
        sentry_ser.setFlowcontrol(serial::flowcontrol_none);
        
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Failed to open serial port: " << e.what());
        return -1;
    }
    
    if (sentry_ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port " << serial_port << " opened successfully");
        
        // 连接CDC设备
        if (!connectCDCDevice()) {
            ROS_WARN("CDC connection check failed, but continuing...");
        }
        
        // 等待设备就绪
        ros::Duration(0.5).sleep();
        
        // 清空缓冲区
        sentry_ser.flush();
        
    } else {
        ROS_ERROR_STREAM("Failed to open serial port");
        return -1;
    }
    
    // 创建发布器
    hp_pub = nh.advertise<std_msgs::UInt16>("robot_hp", 10, true);
    spin_mode_pub = nh.advertise<std_msgs::UInt8>("spin_mode_status", 10, true);
    
    // 创建订阅器
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(
        cmd_vel_topic, 10, sendCmdVel);
    ros::Subscriber spin_sub = nh.subscribe<std_msgs::UInt8>(
        "spin_mode_cmd", 10, spinModeCallback);
    
    ROS_INFO("Sentry serial node initialized - optimized for low latency");
    
    // 启动串口读取线程
    std::thread serial_read_thread(readSerialThread);
    
    // 启动ROS
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    
    // 等待线程结束
    if (serial_read_thread.joinable()) {
        serial_read_thread.join();
    }
    
    // 关闭串口
    if (sentry_ser.isOpen()) {
        sentry_ser.close();
        ROS_INFO("Serial port closed");
    }
    
    return 0;
}