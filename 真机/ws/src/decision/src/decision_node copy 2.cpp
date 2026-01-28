#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/tf.h>
#include <cmath>
#include <mutex>

class DecisionNode {
private:
    ros::NodeHandle nh_;
    
    // 订阅器
    ros::Subscriber hp_sub_;
    ros::Subscriber pose_sub_;
    
    // 发布器
    ros::Publisher goal_pub_;
    ros::Publisher spin_pub_;
    
    // 参数
    double full_hp_threshold_;      // 满血阈值
    double low_hp_threshold_;       // 低血阈值（10%）
    bool enable_spin_at_base_;      // 是否在基地开启小陀螺
    bool enable_stop_spin_when_low_hp_; // 低血时是否停止小陀螺
    std::string team_color_;        // 队伍颜色：red 或 blue
    
    // 目标点
    geometry_msgs::PoseStamped home_pose_;          // 增益区站点
    geometry_msgs::PoseStamped supply_pose_;        // 补给区
    
    // 红蓝方坐标定义
    struct TeamPoses {
        geometry_msgs::Pose red_home;
        geometry_msgs::Pose red_supply;
        geometry_msgs::Pose blue_home;
        geometry_msgs::Pose blue_supply;
    } team_poses_;
    
    // 状态变量
    uint8_t current_hp_;
    uint8_t max_hp_;
    geometry_msgs::Pose current_pose_;
    std::mutex pose_mutex_;
    std::mutex hp_mutex_;
    
    // 状态标志
    bool is_at_home_;
    bool is_at_supply_;
    bool spin_enabled_;
    bool goal_sent_;
    
    // 容差
    double position_tolerance_;
    double orientation_tolerance_;
    
    // 新增：当前目标类型和位置
    enum GoalType {
        GOAL_NONE,
        GOAL_HOME,
        GOAL_SUPPLY
    };
    GoalType current_goal_type_;
    geometry_msgs::PoseStamped current_goal_;

public:
    DecisionNode() : 
        current_hp_(0),
        max_hp_(100),
        is_at_home_(false),
        is_at_supply_(false),
        spin_enabled_(false),
        goal_sent_(false),
        team_color_("red"), // 默认为红方
        current_goal_type_(GOAL_NONE) {
        
        // 初始化参数
        ros::NodeHandle private_nh("~");
        private_nh.param("full_hp_threshold", full_hp_threshold_, 95.0); // 95%以上视为满血
        private_nh.param("low_hp_threshold", low_hp_threshold_, 10.0);   // 10%以下视为低血
        private_nh.param("enable_spin_at_base", enable_spin_at_base_, true);
        private_nh.param("enable_stop_spin_when_low_hp", enable_stop_spin_when_low_hp_, true);
        private_nh.param("position_tolerance", position_tolerance_, 0.1); // 位置容差0.1米
        private_nh.param("orientation_tolerance", orientation_tolerance_, 0.1); // 角度容差0.1弧度
        private_nh.param("team_color", team_color_, team_color_);
        
        // 将队伍颜色转换为小写
        std::transform(team_color_.begin(), team_color_.end(), team_color_.begin(), ::tolower);
        
        // 验证队伍颜色参数
        if (team_color_ != "red" && team_color_ != "blue") {
            // ROS_WARN("Invalid team_color '%s', defaulting to 'red'", team_color_.c_str());
            team_color_ = "red";
        }
        
        // 初始化红蓝方坐标
        initializeTeamPoses();
        
        // 根据队伍颜色设置目标点
        initializeTargetPoses();
        
        // 初始化订阅器
        hp_sub_ = nh_.subscribe<std_msgs::UInt8>("/robot_hp", 10, 
            &DecisionNode::hpCallback, this);
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10,
            &DecisionNode::poseCallback, this);
        
        // 初始化发布器
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, true);
        spin_pub_ = nh_.advertise<std_msgs::Bool>("/spin_mode_cmd", 10, true);
        
        // ROS_INFO("Decision Node initialized");
        // ROS_INFO("Team: %s", team_color_.c_str());
        // ROS_INFO("Full HP threshold: %.1f%%", full_hp_threshold_);
        // ROS_INFO("Low HP threshold: %.1f%%", low_hp_threshold_);
        // ROS_INFO("Home position: (%.2f, %.2f, %.2f)", 
        //          home_pose_.pose.position.x,
        //          home_pose_.pose.position.y,
        //          home_pose_.pose.position.z);
        // ROS_INFO("Supply position: (%.2f, %.2f, %.2f)",
        //          supply_pose_.pose.position.x,
        //          supply_pose_.pose.position.y,
        //          supply_pose_.pose.position.z);
    }
    
    ~DecisionNode() {}
    
    // 初始化红蓝方坐标
    void initializeTeamPoses() {
        // 红方增益区站点
        team_poses_.red_home.position.x = -0.5;
        team_poses_.red_home.position.y = -1;
        team_poses_.red_home.position.z = 0.0;
        team_poses_.red_home.orientation = tf::createQuaternionMsgFromYaw(0.0);
        
        // 红方补给区 (-5.0, 3.0) - 朝向东/右（朝向场地中心）
        team_poses_.red_supply.position.x = -5.0;
        team_poses_.red_supply.position.y = 3.0;
        team_poses_.red_supply.position.z = 0.0;
        team_poses_.red_supply.orientation = tf::createQuaternionMsgFromYaw(0.0);  // 朝向东/右
        
        // 蓝方增益区站点
        team_poses_.blue_home.position.x = 0.5;
        team_poses_.blue_home.position.y = 1;
        team_poses_.blue_home.position.z = 0.0;
        team_poses_.blue_home.orientation = tf::createQuaternionMsgFromYaw(0.0);
        
        // 蓝方补给区 (5.0, -3.0) - 朝向西/左（朝向场地中心）
        team_poses_.blue_supply.position.x = 5.0;
        team_poses_.blue_supply.position.y = -3.0;
        team_poses_.blue_supply.position.z = 0.0;
        team_poses_.blue_supply.orientation = tf::createQuaternionMsgFromYaw(3.1415926535);  // 朝向西/左（π rad）
    }
    
    // 根据队伍颜色初始化目标点
    void initializeTargetPoses() {
        home_pose_.header.frame_id = "map";
        supply_pose_.header.frame_id = "map";
        
        if (team_color_ == "red") {
            home_pose_.pose = team_poses_.red_home;
            supply_pose_.pose = team_poses_.red_supply;
        } else { // blue
            home_pose_.pose = team_poses_.blue_home;
            supply_pose_.pose = team_poses_.blue_supply;
        }
    }
    
    // 血量回调函数
    void hpCallback(const std_msgs::UInt8::ConstPtr& msg) {
        // std::lock_guard<std::mutex> lock(hp_mutex_);
        current_hp_ = msg->data;

        // ROS_INFO("Current HP: %d", current_hp_);
        
        // 决策逻辑
        makeDecision();
    }
    
    // 位姿回调函数
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        // std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_ = msg->pose.pose;
        
        // 检查是否到达目标点
        checkPosition();
        
        // 每次更新位姿都重新决策
        makeDecision();
    }
    
    // 检查当前位置
    void checkPosition() {
        // 检查是否在增益区
        if (isAtPosition(current_pose_, home_pose_.pose)) {
            if (!is_at_home_) {
                // ROS_INFO("Arrived at %s home position (gain zone)", team_color_.c_str());
                is_at_home_ = true;
                goal_sent_ = false;
                // 到达目标点，清空当前目标
                if (current_goal_type_ == GOAL_HOME) {
                    current_goal_type_ = GOAL_NONE;
                }
            }
        } else {
            is_at_home_ = false;
        }
        
        // 检查是否在补给区
        if (isAtPosition(current_pose_, supply_pose_.pose)) {
            if (!is_at_supply_) {
                // ROS_INFO("Arrived at %s supply position", team_color_.c_str());
                is_at_supply_ = true;
                goal_sent_ = false;
                // 到达目标点，清空当前目标
                if (current_goal_type_ == GOAL_SUPPLY) {
                    current_goal_type_ = GOAL_NONE;
                }
            }
        } else {
            is_at_supply_ = false;
        }
    }
    
    // 判断是否在指定位置（带容差）
    bool isAtPosition(const geometry_msgs::Pose& current, const geometry_msgs::Pose& target) {
        // 检查位置
        double dx = current.position.x - target.position.x;
        double dy = current.position.y - target.position.y;
        double dz = current.position.z - target.position.z;
        double distance = sqrt(dx*dx + dy*dy + dz*dz);
        
        // 检查朝向
        double current_yaw = tf::getYaw(current.orientation);
        double target_yaw = tf::getYaw(target.orientation);
        double yaw_diff = fabs(current_yaw - target_yaw);
        
        // 归一化角度差到 [0, PI]
        if (yaw_diff > M_PI) {
            yaw_diff = 2 * M_PI - yaw_diff;
        }
        
        return (distance < position_tolerance_) && (yaw_diff < orientation_tolerance_);
    }
    
    // 判断两个目标是否相同
    bool isSameGoal(const geometry_msgs::PoseStamped& goal1, const geometry_msgs::PoseStamped& goal2) {
        if (goal1.header.frame_id != goal2.header.frame_id) {
            return false;
        }
        
        double dx = goal1.pose.position.x - goal2.pose.position.x;
        double dy = goal1.pose.position.y - goal2.pose.position.y;
        double dz = goal1.pose.position.z - goal2.pose.position.z;
        double distance = sqrt(dx*dx + dy*dy + dz*dz);
        
        double yaw1 = tf::getYaw(goal1.pose.orientation);
        double yaw2 = tf::getYaw(goal2.pose.orientation);
        double yaw_diff = fabs(yaw1 - yaw2);
        if (yaw_diff > M_PI) {
            yaw_diff = 2 * M_PI - yaw_diff;
        }
        
        return (distance < 0.01) && (yaw_diff < 0.01); // 更小的容差来判断是否相同
    }
    
    // 决策逻辑
    void makeDecision() {
        // std::lock_guard<std::mutex> lock1(hp_mutex_);
        // std::lock_guard<std::mutex> lock2(pose_mutex_);
        
        double hp_percentage = (static_cast<double>(current_hp_) / max_hp_) * 100.0;
        
        // ROS_INFO("HP percentage: %.1f%%, At home: %s, At supply: %s", 
        //          hp_percentage, is_at_home_ ? "true" : "false", is_at_supply_ ? "true" : "false");
        
        // 情况1：血量满（>95%）且不在增益区
        if (hp_percentage >= full_hp_threshold_) {
            if (!is_at_home_) {
                // 检查是否需要发送新目标
                if (current_goal_type_ != GOAL_HOME) {
                    // ROS_INFO("HP is full (%.1f%%). Sending to %s gain zone...", 
                            //  hp_percentage, team_color_.c_str());
                    sendGoal(home_pose_, GOAL_HOME);
                    current_goal_type_ = GOAL_HOME;
                    goal_sent_ = true;
                } else {
                    // ROS_INFO("Already heading to gain zone");
                }
            } else {
                // 如果在增益区，清空当前目标
                if (current_goal_type_ != GOAL_NONE) {
                    current_goal_type_ = GOAL_NONE;
                }
            }
            
            // 如果在增益区且允许开启小陀螺
            if (is_at_home_ && enable_spin_at_base_ && !spin_enabled_) {
                // ROS_INFO("At %s gain zone with full HP. Enabling spin mode...", team_color_.c_str());
                enableSpinMode(true);
                spin_enabled_ = true;
            }
        }
        // 情况2：血量低（<10%）
        else if (hp_percentage <= low_hp_threshold_) {
            // 如果低血时应该停止小陀螺
            if (enable_stop_spin_when_low_hp_ && spin_enabled_) {
                // ROS_INFO("HP is low (%.1f%%). Stopping spin mode...", hp_percentage);
                enableSpinMode(false);
                spin_enabled_ = false;
            }
            
            // 如果不在补给区
            if (!is_at_supply_) {
                // 检查是否需要发送新目标
                if (current_goal_type_ != GOAL_SUPPLY) {
                    // ROS_INFO("HP is low (%.1f%%). Sending to %s supply zone...", 
                            //  hp_percentage, team_color_.c_str());
                    sendGoal(supply_pose_, GOAL_SUPPLY);
                    current_goal_type_ = GOAL_SUPPLY;
                    goal_sent_ = true;
                } else {
                    // ROS_INFO("Already heading to supply zone");
                }
            } else {
                // 如果在补给区，清空当前目标
                if (current_goal_type_ != GOAL_NONE) {
                    current_goal_type_ = GOAL_NONE;
                }
            }
        }
        // 情况3：中等血量
        else {
            // 可以添加其他决策逻辑
            // ROS_INFO("HP is at medium level (%.1f%%)", hp_percentage);
        }
    }
    
    // 修改sendGoal函数
    void sendGoal(const geometry_msgs::PoseStamped& goal, GoalType goal_type) {
        // 只在实际发送时才更新时间戳
        geometry_msgs::PoseStamped new_goal = goal;
        new_goal.header.stamp = ros::Time::now();
        
        goal_pub_.publish(new_goal);
        
        // 更新当前目标
        current_goal_ = new_goal;
        
        // ROS_INFO("Goal sent: type=%d, frame=%s, stamp=%.6f, pos=(%.2f, %.2f, %.2f)", 
                // goal_type,
                // new_goal.header.frame_id.c_str(),
                // new_goal.header.stamp.toSec(),
                // new_goal.pose.position.x, 
                // new_goal.pose.position.y,
                // new_goal.pose.position.z);
    }
    
    // 控制小陀螺模式
    void enableSpinMode(bool enable) {
        std_msgs::Bool msg;
        msg.data = enable;
        spin_pub_.publish(msg);
        // ROS_INFO("Spin mode %s", enable ? "enabled" : "disabled");
    }
    
    void run() {
        ros::Rate rate(10); // 10Hz
        
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "decision_node");
    
    DecisionNode node;
    node.run();
    return 0;
}