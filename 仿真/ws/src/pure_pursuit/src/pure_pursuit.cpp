#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <nav_msgs/Odometry.h>  // 添加这个头文件

class PurePursuitController {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber path_sub_;
    ros::Subscriber odom_sub_;
    
    // 发布者
    ros::Publisher cmd_vel_pub_;
    ros::Publisher lookahead_pub_;
    
    // TF监听器
    tf::TransformListener tf_listener_;
    
    // 控制参数
    double lookahead_distance_;
    double linear_velocity_;
    double max_linear_vel_;
    double min_linear_vel_;
    double max_angular_vel_;
    double wheel_base_;
    double goal_tolerance_;
    bool use_fixed_velocity_;
    
    // 状态变量
    geometry_msgs::PoseStamped robot_pose_;
    nav_msgs::Path current_path_;
    bool path_received_;
    bool odom_received_;
    
public:
    PurePursuitController() : 
        private_nh_("~"),
        path_received_(false),
        odom_received_(false) {
        
        // 初始化参数
        initParams();
        
        // 初始化订阅者和发布者
        path_sub_ = nh_.subscribe("/move_base/NavfnROS/plan", 1, &PurePursuitController::pathCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 1, &PurePursuitController::odomCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel1", 1);
        lookahead_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/lookahead_point", 1);
        
        ROS_INFO("Pure Pursuit Controller initialized");
    }
    
    void initParams() {
        private_nh_.param("lookahead_distance", lookahead_distance_, 0.5);
        private_nh_.param("linear_velocity", linear_velocity_, 0.3);
        private_nh_.param("max_linear_vel", max_linear_vel_, 0.5);
        private_nh_.param("min_linear_vel", min_linear_vel_, 0.1);
        private_nh_.param("max_angular_vel", max_angular_vel_, 1.0);
        private_nh_.param("wheel_base", wheel_base_, 0.3);
        private_nh_.param("goal_tolerance", goal_tolerance_, 0.1);
        private_nh_.param("use_fixed_velocity", use_fixed_velocity_, true);
        
        ROS_INFO("Pure Pursuit Parameters:");
        ROS_INFO("  lookahead_distance: %.2f", lookahead_distance_);
        ROS_INFO("  linear_velocity: %.2f", linear_velocity_);
        ROS_INFO("  max_angular_vel: %.2f", max_angular_vel_);
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        robot_pose_.header = msg->header;
        robot_pose_.pose = msg->pose.pose;
        odom_received_ = true;
    }
    
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        if (msg->poses.empty()) {
            ROS_WARN("Received empty path!");
            return;
        }
        
        current_path_ = *msg;
        path_received_ = true;
        ROS_INFO("Received path with %zu points", current_path_.poses.size());
    }
    
    int findClosestPoint(const geometry_msgs::PoseStamped& robot_pose, 
                        const nav_msgs::Path& path) {
        if (path.poses.empty()) return -1;
        
        int closest_idx = 0;
        double min_distance = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < path.poses.size(); i++) {
            double dx = path.poses[i].pose.position.x - robot_pose.pose.position.x;
            double dy = path.poses[i].pose.position.y - robot_pose.pose.position.y;
            double distance = std::hypot(dx, dy);
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_idx = i;
            }
        }
        
        return closest_idx;
    }
    
    geometry_msgs::PoseStamped getLookaheadPoint(const geometry_msgs::PoseStamped& robot_pose, 
                                                const nav_msgs::Path& path) {
        geometry_msgs::PoseStamped lookahead_pose;
        
        if (path.poses.empty()) {
            lookahead_pose = robot_pose;
            return lookahead_pose;
        }
        
        // 找到最近点
        int closest_idx = findClosestPoint(robot_pose, path);
        if (closest_idx < 0) {
            lookahead_pose = robot_pose;
            return lookahead_pose;
        }
        
        // 寻找前视点
        for (size_t i = closest_idx; i < path.poses.size(); i++) {
            double dx = path.poses[i].pose.position.x - robot_pose.pose.position.x;
            double dy = path.poses[i].pose.position.y - robot_pose.pose.position.y;
            double distance = std::hypot(dx, dy);
            
            if (distance >= lookahead_distance_) {
                lookahead_pose = path.poses[i];
                // 发布前视点用于可视化
                lookahead_pub_.publish(lookahead_pose);
                return lookahead_pose;
            }
        }
        
        // 如果找不到，返回最后一个点
        lookahead_pose = path.poses.back();
        lookahead_pub_.publish(lookahead_pose);
        return lookahead_pose;
    }
    
    double calculateCurvature(const geometry_msgs::PoseStamped& robot_pose,
                            const geometry_msgs::PoseStamped& lookahead_pose) {
        // 计算目标点在机器人坐标系中的位置
        double dx = lookahead_pose.pose.position.x - robot_pose.pose.position.x;
        double dy = lookahead_pose.pose.position.y - robot_pose.pose.position.y;
        
        // 转换到机器人坐标系
        double robot_yaw = tf::getYaw(robot_pose.pose.orientation);
        double local_x = dx * std::cos(robot_yaw) + dy * std::sin(robot_yaw);
        double local_y = -dx * std::sin(robot_yaw) + dy * std::cos(robot_yaw);
        
        // 计算曲率（纯追踪公式）
        double distance_sq = local_x * local_x + local_y * local_y;
        if (distance_sq < 0.001) return 0.0;
        
        return 2.0 * local_y / distance_sq;
    }
    
    double calculateAdaptiveVelocity(double distance_to_goal, double curvature) {
        if (use_fixed_velocity_) {
            return linear_velocity_;
        }
        
        // 自适应速度：根据到目标的距离和路径曲率调整
        double velocity = linear_velocity_;
        
        // 接近目标时减速
        if (distance_to_goal < 1.0) {
            velocity *= (distance_to_goal / 1.0);
        }
        
        // 急转弯时减速
        double abs_curvature = std::abs(curvature);
        if (abs_curvature > 0.5) {
            velocity *= (0.5 / abs_curvature);
        }
        
        // 限制速度范围
        velocity = std::max(min_linear_vel_, std::min(velocity, max_linear_vel_));
        
        return velocity;
    }
    
    void controlLoop() {
        if (!path_received_ || !odom_received_) {
            return;
        }
        
        if (current_path_.poses.empty()) {
            ROS_WARN_THROTTLE(1, "No path available");
            stopRobot();
            return;
        }
        
        // 获取前视点
        geometry_msgs::PoseStamped lookahead_pose = getLookaheadPoint(robot_pose_, current_path_);
        
        // 检查是否到达目标
        geometry_msgs::PoseStamped goal_pose = current_path_.poses.back();
        double dx_to_goal = goal_pose.pose.position.x - robot_pose_.pose.position.x;
        double dy_to_goal = goal_pose.pose.position.y - robot_pose_.pose.position.y;
        double distance_to_goal = std::hypot(dx_to_goal, dy_to_goal);
        
        if (distance_to_goal < goal_tolerance_) {
            ROS_INFO("Goal reached!");
            stopRobot();
            return;
        }
        
        // 计算曲率
        double curvature = calculateCurvature(robot_pose_, lookahead_pose);
        
        // 计算速度
        double linear_vel = calculateAdaptiveVelocity(distance_to_goal, curvature);
        
        // 计算角速度
        double angular_vel = 0.0;
        if (wheel_base_ > 0) {
            // 对于车辆模型：angular = linear * tan(steering) / wheel_base
            // steering = atan(curvature * wheel_base)
            double steering_angle = std::atan(curvature * wheel_base_);
            angular_vel = linear_vel * std::tan(steering_angle) / wheel_base_;
        } else {
            // 对于差速机器人：angular = linear * curvature
            angular_vel = linear_vel * curvature;
        }
        
        // 限制角速度
        angular_vel = std::max(-max_angular_vel_, std::min(angular_vel, max_angular_vel_));
        
        // 发布控制命令
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;
        cmd_vel_pub_.publish(cmd_vel);
        
        ROS_INFO_THROTTLE(1, "PP Control: v=%.2f, w=%.2f, curv=%.2f, dist=%.2f", 
                         linear_vel, angular_vel, curvature, distance_to_goal);
    }
    
    void stopRobot() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_vel);
    }
    
    void run() {
        ros::Rate rate(20); // 20Hz
        
        while (ros::ok()) {
            controlLoop();
            ros::spinOnce();
            rate.sleep();
        }
        
        stopRobot();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_controller");
    PurePursuitController controller;
    controller.run();
    return 0;
}