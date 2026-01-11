#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

class TestNode
{
private:
    ros::NodeHandle nh_;
    
    // 发布器
    ros::Publisher cmd_vel_pub_;
    ros::Publisher spin_mode_pub_;
    
    // 订阅器
    ros::Subscriber hp_sub_;
    ros::Subscriber spin_status_sub_;
    
    bool spin_enabled_;
    ros::Timer timer_;

public:
    TestNode() : spin_enabled_(false)
    {
        // 初始化发布器
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        spin_mode_pub_ = nh_.advertise<std_msgs::Bool>("spin_mode_cmd", 10);
        
        // 初始化订阅器
        hp_sub_ = nh_.subscribe<std_msgs::UInt8>(
            "robot_hp", 10, &TestNode::hpCallback, this);
        spin_status_sub_ = nh_.subscribe<std_msgs::Bool>(
            "spin_mode_status", 10, &TestNode::spinStatusCallback, this);
        
        // 创建定时器，1Hz频率
        timer_ = nh_.createTimer(ros::Duration(1.0), &TestNode::timerCallback, this);
        
        ROS_INFO("Test node initialized");
    }
    
    ~TestNode() {}
    
    // 血量数据回调函数
    void hpCallback(const std_msgs::UInt8::ConstPtr& msg)
    {
        ROS_INFO("Robot HP: %d", msg->data);
    }
    
    // 小陀螺状态回调函数
    void spinStatusCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        ROS_INFO("Spin mode status: %s", msg->data ? "true" : "false");
    }
    
    // 定时器回调函数
    void timerCallback(const ros::TimerEvent& event)
    {
        // 发布cmd_vel消息
        geometry_msgs::Twist twist;
        twist.linear.x = 0.5;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.2;
        
        cmd_vel_pub_.publish(twist);
        
        // 切换并发布小陀螺状态
        spin_enabled_ = !spin_enabled_;
        std_msgs::Bool spin_msg;
        spin_msg.data = spin_enabled_;
        spin_mode_pub_.publish(spin_msg);
        
        ROS_INFO("Published cmd_vel and spin mode: %s", spin_enabled_ ? "true" : "false");
    }
    
    void run()
    {
        ros::spin();
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_node");
    
    TestNode node;
    node.run();
    
    return 0;
}