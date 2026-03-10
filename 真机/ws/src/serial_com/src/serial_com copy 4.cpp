#include "ros/ros.h"
#include "serial_com.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <string>
#include <iostream>
#include <unistd.h> // for usleep


// 新的数据帧结构体
#pragma pack(push, 1)
struct FrameHeader {
    uint8_t start1 = 0xFA;
    uint8_t start2 = 0xFB;
    uint8_t cmd_id;      // 指令ID
    uint8_t data_len;    // 数据长度
};

// 雷达发送帧 (雷达 -> PC -> 电控)
struct RadarSendFrame {
    FrameHeader header;  // FA, FB, 0x21, 13
    float linear_x;
    float linear_y;
    float angular_z;
    uint8_t spin_mode;
};

// 雷达接收帧 (电控 -> PC -> 雷达)
struct RadarRecvFrame {
    FrameHeader header;  // FA, FB, 0x41, 2 (修改：数据长度改为2，因为hp现在是uint16_t)
    uint16_t hp;         // 修改：改为uint16_t以支持最高400血量
};
#pragma pack(pop)


// 全局变量
serial::Serial sentry_ser;                  // 串口对象
std::string cmd_vel_topic;                  // 话题名
std::atomic<uint8_t> spin_mode_enabled(0);  // 使用原子变量确保线程安全，0:关闭，1:开启
ros::Publisher hp_pub;                      // 血量发布器
ros::Publisher spin_mode_pub;               // 小陀螺状态发布器

// 小陀螺状态回调函数
void spinModeCallback(const std_msgs::UInt8::ConstPtr &msg)
{
    uint8_t new_spin_mode = msg->data;
    
    spin_mode_enabled.store(new_spin_mode);
    
    // 立即发布状态到spin_mode_status
    std_msgs::UInt8 status_msg;
    status_msg.data = new_spin_mode;
    spin_mode_pub.publish(status_msg);
    
    // ROS_INFO("Spin mode command received: %d, published to spin_mode_status", new_spin_mode);
}

// 接收到订阅的消息后，会进入消息回调函数
// cmd_vel消息的回调函数，收到速度指令后打印线速度和角速度
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    // 获取当前小陀螺状态
    uint8_t current_spin_mode = spin_mode_enabled.load();
    
    // 发送雷达发送帧
    RadarSendFrame frame;
    frame.header.start1 = 0xFA;
    frame.header.start2 = 0xFB;
    frame.header.cmd_id = 0x21;
    frame.header.data_len = sizeof(frame.linear_x) + sizeof(frame.linear_y) + sizeof(frame.angular_z) + sizeof(frame.spin_mode);
    frame.linear_x = std::round(msg->linear.x * 1000.0f) / 1000.0f;
    frame.linear_y = std::round(msg->linear.y * 1000.0f) / 1000.0f;
    frame.angular_z = std::round(msg->angular.z * 1000.0f) / 1000.0f;
    frame.spin_mode = current_spin_mode;

    // 修改现有的发送代码
    try {
        if (sentry_ser.isOpen()) {
            sentry_ser.flush();
            
            // 打印数据（发送前）
            uint8_t* data = reinterpret_cast<uint8_t*>(&frame);
            std::stringstream ss;
            ss << "[TX] Sending: ";
            for (size_t i = 0; i < sizeof(RadarSendFrame); i++) {
                ss << std::hex << std::setw(2) << std::setfill('0') 
                << static_cast<int>(data[i]) << " ";
            }
            ROS_INFO("%s", ss.str().c_str());
            
            size_t bytes_written = sentry_ser.write(data, sizeof(RadarSendFrame));
            
            if (bytes_written == sizeof(RadarSendFrame)) {
                ROS_INFO("[TX] RadarSendFrame: linear_x=%.4f linear_y=%.4f angular_z=%.4f spin_mode=%d", 
                        frame.linear_x, frame.linear_y, frame.angular_z, frame.spin_mode);
            } else {
                ROS_WARN("[TX] Only wrote %zu bytes, expected %zu", bytes_written, sizeof(RadarSendFrame));
            }
        } else {
            ROS_ERROR("Serial port not open!");
        }
    } catch (serial::IOException &e) {
        ROS_ERROR("Serial write error: %s", e.what());
    }

    // 注意：这里不再发布spin_mode_status，因为在spinModeCallback中已经发布了
}

// 读取串口数据并处理帧的线程函数
void readSerialThread()
{
    ROS_INFO("Serial read thread started");
    uint8_t buffer[256];
    size_t buffer_index = 0;
    bool frame_started = false;
    
    while (ros::ok()) {
        try {
            if (sentry_ser.isOpen()) {
                // 读取所有可用数据
                size_t bytes_available = sentry_ser.available();
                if (bytes_available > 0) {
                    uint8_t temp_buffer[256];
                    size_t bytes_read = sentry_ser.read(temp_buffer, 
                        bytes_available > sizeof(temp_buffer) ? sizeof(temp_buffer) : bytes_available);
                    
                    // 处理读取到的每个字节
                    for (size_t i = 0; i < bytes_read; i++) {
                        uint8_t current_byte = temp_buffer[i];
                        
                        if (!frame_started) {
                            // 寻找帧起始符 0xFA
                            if (current_byte == 0xFA) {
                                buffer_index = 0;
                                buffer[buffer_index++] = current_byte;
                                frame_started = true;
                            }
                        } else {
                            // 收集帧数据
                            buffer[buffer_index++] = current_byte;
                            
                            // 检查第二个字节是否为 0xFB
                            if (buffer_index == 2) {
                                if (current_byte != 0xFB) {
                                    // 不是有效的帧起始
                                    frame_started = false;
                                    buffer_index = 0;
                                    ROS_WARN("[RX] Invalid second start byte: 0x%02X", current_byte);
                                    continue;
                                }
                            }
                            
                            // 检查是否收到完整帧（6字节）
                            if (buffer_index >= sizeof(RadarRecvFrame)) {
                                // 直接处理帧数据
                                RadarRecvFrame recv_frame;
                                memcpy(&recv_frame, buffer, sizeof(RadarRecvFrame));
                                
                                // 验证帧起始位
                                if (recv_frame.header.start1 == 0xFA && recv_frame.header.start2 == 0xFB) {
                                    // 打印原始数据（用于调试）
                                    ROS_INFO("[RX] Raw data: %02X %02X %02X %02X %02X %02X", 
                                            buffer[0], buffer[1], buffer[2], buffer[3], 
                                            buffer[4], buffer[5]);
                                    
                                    // 处理命令ID
                                    if (recv_frame.header.cmd_id == 0x41) {
                                        std_msgs::UInt16 hp_msg;
                                        hp_msg.data = recv_frame.hp;
                                        hp_pub.publish(hp_msg);
                                        ROS_INFO("[RX] Vision HP: %d", recv_frame.hp);
                                    } else {
                                        ROS_WARN("[RX] Invalid cmd_id: 0x%02X", recv_frame.header.cmd_id);
                                    }
                                } else {
                                    ROS_WARN("[RX] Invalid start bytes: 0x%02X 0x%02X", 
                                            recv_frame.header.start1, recv_frame.header.start2);
                                }
                                
                                // 重置状态，准备下一帧
                                frame_started = false;
                                buffer_index = 0;
                            }
                            
                            // 防止缓冲区溢出
                            if (buffer_index >= sizeof(buffer)) {
                                ROS_WARN("[RX] Buffer overflow, resetting frame detection");
                                frame_started = false;
                                buffer_index = 0;
                            }
                        }
                    }
                }
            }
        } catch (serial::IOException &e) {
            ROS_ERROR("Serial read error: %s", e.what());
            ros::Duration(0.1).sleep();
        }
        usleep(10000);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_com"); // ros::init 初始化ROS节点，节点名为 serial_com。
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 获取参数
    private_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic, "/cmd_vel1");
    std::string serial_port;
    private_nh.param<std::string>("serial_port", serial_port, "/dev/ttyACM0");

    ROS_INFO("Using serial port: %s", serial_port.c_str());
    ROS_INFO("Subscribing to topic: %s", cmd_vel_topic.c_str());
    // 打印当前使用的串口和订阅的话题名。

    // 设置串口
    try
    {
        sentry_ser.setPort(serial_port);                           // 设置串口端口。
        sentry_ser.setBaudrate(115200);                            // 设置串口波特率为115200。
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); // 设置串口超时时间为1000毫秒。
        sentry_ser.setTimeout(to);                                 // 设置串口超时时间。
        sentry_ser.open();                                         // 打开串口。

        // 配置串口参数（可选）
        sentry_ser.setParity(serial::parity_none);    // 设置无奇偶校验。
        sentry_ser.setStopbits(serial::stopbits_one); // 设置1个停止位。
        sentry_ser.setBytesize(serial::eightbits);    // 设置8个数据位。
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port " << serial_port << ": " << e.what());
        return -1;
    }
    catch (serial::SerialException &e)
    {
        ROS_ERROR_STREAM("Serial exception: " << e.what());
        return -1;
    }
    catch (...)
    {
        ROS_ERROR_STREAM("Unknown error opening serial port");
        return -1;
    }

    if (sentry_ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port " << serial_port << " opened successfully");
    }
    else
    {
        ROS_ERROR_STREAM("Failed to open serial port");
        return -1;
    }
    // 如果串口成功打开，打印成功信息；否则打印错误信息并退出程序。

    // 创建发布器，分别用于发布机器人血量和小陀螺状态
    hp_pub = nh.advertise<std_msgs::UInt16>("robot_hp", 10, true);  // 修改：使用UInt16发布器
    spin_mode_pub = nh.advertise<std_msgs::UInt8>("spin_mode_status", 10, true);  // 改为UInt8

    // 创建订阅器，分别订阅速度指令和小陀螺控制指令，收到消息后会调用对应的回调函数。
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(
        cmd_vel_topic, 10, cmdVelCallback);
    ros::Subscriber spin_sub = nh.subscribe<std_msgs::UInt8>(  // 改为UInt8
        "spin_mode_cmd", 10, spinModeCallback);

    ROS_INFO("Sentry serial node initialized");
    // 打印节点初始化完成的信息。

    // 启动串口读取线程，专门负责串口数据的读取。
    std::thread serial_read_thread(readSerialThread);

    // 启动ROS多线程主循环，允许回调和串口线程并发执行。
    ros::MultiThreadedSpinner spinner(2); // 使用多线程spinner
    spinner.spin();

    // 等待线程结束
    if (serial_read_thread.joinable())
    {
        serial_read_thread.join();
    }

    // 关闭串口
    if (sentry_ser.isOpen())
    {
        sentry_ser.close();
        ROS_INFO("Serial port closed");
    }

    return 0;
}