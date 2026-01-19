#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>  // 添加这个头文件
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <angles/angles.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

class SmoothPathFollower {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber pose_sub_;
    ros::Subscriber path_sub_;
    
    // 发布者
    ros::Publisher cmd_vel_pub_;
    
    // TF监听器
    tf::TransformListener tf_listener_;
    
    // 控制参数
    double max_linear_vel_;
    double max_angular_vel_;
    double linear_kp_;
    double angular_kp_;
    double lookahead_distance_;
    double goal_tolerance_;
    double max_acceleration_;
    double max_deceleration_;
    
    // 状态变量
    geometry_msgs::PoseStamped current_pose_;
    nav_msgs::Path current_path_;
    bool pose_received_;
    bool path_received_;
    ros::Time last_control_time_;
    double current_linear_vel_;
    double current_angular_vel_;
    
    // 路径平滑参数
    double path_smoothing_factor_;
    int interpolation_points_;
    
public:
    SmoothPathFollower() : 
        private_nh_("~"),
        pose_received_(false),
        path_received_(false),
        current_linear_vel_(0.0),
        current_angular_vel_(0.0) {
        
        // 初始化参数
        initParams();
        
        // 初始化订阅者和发布者
        pose_sub_ = nh_.subscribe("/amcl_pose", 1, &SmoothPathFollower::poseCallback, this);
        path_sub_ = nh_.subscribe("/move_base/NavfnROS/plan", 1, &SmoothPathFollower::pathCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel1", 1);
        
        last_control_time_ = ros::Time::now();
        
        ROS_INFO("Smooth Path Follower initialized");
    }
    
    void initParams() {
        private_nh_.param("max_linear_vel", max_linear_vel_, 0.3);
        private_nh_.param("max_angular_vel", max_angular_vel_, 0.5);
        private_nh_.param("linear_kp", linear_kp_, 1.0);
        private_nh_.param("angular_kp", angular_kp_, 2.0);
        private_nh_.param("lookahead_distance", lookahead_distance_, 0.3);
        private_nh_.param("goal_tolerance", goal_tolerance_, 0.1);
        private_nh_.param("max_acceleration", max_acceleration_, 0.5);
        private_nh_.param("max_deceleration", max_deceleration_, 0.8);
        private_nh_.param("path_smoothing_factor", path_smoothing_factor_, 0.3);
        private_nh_.param("interpolation_points", interpolation_points_, 5);
        
        ROS_INFO("Parameters loaded: max_linear_vel=%.2f, max_angular_vel=%.2f", 
                 max_linear_vel_, max_angular_vel_);
    }
    
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        current_pose_.header = msg->header;
        current_pose_.pose = msg->pose.pose;
        pose_received_ = true;
    }
    
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        if (msg->poses.empty()) {
            ROS_WARN("Received empty path!");
            return;
        }
        
        // 平滑路径
        current_path_ = smoothPath(*msg);
        path_received_ = true;
        
        ROS_INFO("Received new path with %zu points", current_path_.poses.size());
    }
    
    nav_msgs::Path smoothPath(const nav_msgs::Path& path) {
        if (path.poses.size() < 3) {
            return path;
        }
        
        nav_msgs::Path smoothed_path = path;
        
        // 简单的移动平均平滑
        for (size_t i = 1; i < path.poses.size() - 1; i++) {
            smoothed_path.poses[i].pose.position.x = 
                path_smoothing_factor_ * path.poses[i].pose.position.x +
                (1 - path_smoothing_factor_) * 0.5 * 
                (path.poses[i-1].pose.position.x + path.poses[i+1].pose.position.x);
            
            smoothed_path.poses[i].pose.position.y = 
                path_smoothing_factor_ * path.poses[i].pose.position.y +
                (1 - path_smoothing_factor_) * 0.5 * 
                (path.poses[i-1].pose.position.y + path.poses[i+1].pose.position.y);
        }
        
        // 插值增加路径点
        return interpolatePath(smoothed_path);
    }
    
    nav_msgs::Path interpolatePath(const nav_msgs::Path& path) {
        if (path.poses.size() < 2) {
            return path;
        }
        
        nav_msgs::Path interpolated_path;
        interpolated_path.header = path.header;
        
        for (size_t i = 0; i < path.poses.size() - 1; i++) {
            // 添加当前点
            interpolated_path.poses.push_back(path.poses[i]);
            
            // 在当前点和下一个点之间插值
            for (int j = 1; j <= interpolation_points_; j++) {
                double alpha = static_cast<double>(j) / (interpolation_points_ + 1);
                
                geometry_msgs::PoseStamped interpolated_pose;
                interpolated_pose.header = path.header;
                
                // 线性插值位置
                interpolated_pose.pose.position.x = 
                    (1 - alpha) * path.poses[i].pose.position.x + 
                    alpha * path.poses[i+1].pose.position.x;
                
                interpolated_pose.pose.position.y = 
                    (1 - alpha) * path.poses[i].pose.position.y + 
                    alpha * path.poses[i+1].pose.position.y;
                
                // 角度插值
                double yaw1 = tf::getYaw(path.poses[i].pose.orientation);
                double yaw2 = tf::getYaw(path.poses[i+1].pose.orientation);
                double interp_yaw = (1 - alpha) * yaw1 + alpha * yaw2;
                
                interpolated_pose.pose.orientation = 
                    tf::createQuaternionMsgFromYaw(interp_yaw);
                
                interpolated_path.poses.push_back(interpolated_pose);
            }
        }
        
        // 添加最后一个点
        interpolated_path.poses.push_back(path.poses.back());
        
        return interpolated_path;
    }
    
    geometry_msgs::PoseStamped getLookaheadPoint(const geometry_msgs::PoseStamped& robot_pose, 
                                                const nav_msgs::Path& path) {
        if (path.poses.empty()) {
            return robot_pose;
        }
        
        // 找到距离机器人最近的路点
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
        
        // 寻找前视点
        for (size_t i = closest_idx; i < path.poses.size(); i++) {
            double dx = path.poses[i].pose.position.x - robot_pose.pose.position.x;
            double dy = path.poses[i].pose.position.y - robot_pose.pose.position.y;
            double distance = std::hypot(dx, dy);
            
            if (distance >= lookahead_distance_) {
                return path.poses[i];
            }
        }
        
        // 如果找不到合适的前视点，返回最后一个点
        return path.poses.back();
    }
    
    double calculateCurvature(const geometry_msgs::PoseStamped& robot_pose,
                            const geometry_msgs::PoseStamped& target_pose) {
        // 计算目标点在机器人坐标系中的位置
        double dx = target_pose.pose.position.x - robot_pose.pose.position.x;
        double dy = target_pose.pose.position.y - robot_pose.pose.position.y;
        
        // 转换到机器人坐标系
        double robot_yaw = tf::getYaw(robot_pose.pose.orientation);
        double local_x = dx * std::cos(robot_yaw) + dy * std::sin(robot_yaw);
        double local_y = -dx * std::sin(robot_yaw) + dy * std::cos(robot_yaw);
        
        // 计算曲率（纯追踪算法）
        if (std::abs(local_x * local_x + local_y * local_y) < 0.001) {
            return 0.0;
        }
        double curvature = 2.0 * local_y / (local_x * local_x + local_y * local_y);
        
        return curvature;
    }
    
    // 自定义clamp函数，替代C++17的std::clamp
    double clamp(double value, double min_val, double max_val) {
        if (value < min_val) return min_val;
        if (value > max_val) return max_val;
        return value;
    }
    
    void limitVelocity(double& linear_vel, double& angular_vel) {
        // 限制最大速度
        linear_vel = clamp(linear_vel, -max_linear_vel_, max_linear_vel_);
        angular_vel = clamp(angular_vel, -max_angular_vel_, max_angular_vel_);
        
        // 考虑非完整约束（防止同时高速直线和旋转）
        if (std::abs(angular_vel) > max_angular_vel_ * 0.8) {
            linear_vel *= 0.5;
        }
    }
    
    void applyAccelerationLimits(double& linear_vel, double& angular_vel) {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_control_time_).toSec();
        
        if (dt <= 0) {
            return;
        }
        
        // 线性加速度限制
        double linear_acc = (linear_vel - current_linear_vel_) / dt;
        double max_acc = (linear_vel > current_linear_vel_) ? max_acceleration_ : max_deceleration_;
        if (std::abs(linear_acc) > max_acc) {
            linear_vel = current_linear_vel_ + std::copysign(max_acc * dt, linear_acc);
        }
        
        // 角加速度限制
        double angular_acc = (angular_vel - current_angular_vel_) / dt;
        if (std::abs(angular_acc) > max_acceleration_ * 2) {
            angular_vel = current_angular_vel_ + std::copysign(max_acceleration_ * 2 * dt, angular_acc);
        }
        
        last_control_time_ = current_time;
        current_linear_vel_ = linear_vel;
        current_angular_vel_ = angular_vel;
    }
    
    void controlLoop() {
        if (!pose_received_ || !path_received_) {
            return;
        }
        
        if (current_path_.poses.empty()) {
            ROS_WARN_THROTTLE(1, "No path available");
            stopRobot();
            return;
        }
        
        // 获取前视点
        geometry_msgs::PoseStamped lookahead_pose = getLookaheadPoint(current_pose_, current_path_);
        
        // 计算到终点的距离
        geometry_msgs::PoseStamped goal_pose = current_path_.poses.back();
        double dx_to_goal = goal_pose.pose.position.x - current_pose_.pose.position.x;
        double dy_to_goal = goal_pose.pose.position.y - current_pose_.pose.position.y;
        double distance_to_goal = std::hypot(dx_to_goal, dy_to_goal);
        
        // 检查是否到达目标
        if (distance_to_goal < goal_tolerance_) {
            ROS_INFO("Goal reached!");
            stopRobot();
            return;
        }
        
        // 计算控制命令
        double linear_vel = 0.0;
        double angular_vel = 0.0;
        
        // 计算到前视点的距离和角度误差
        double dx = lookahead_pose.pose.position.x - current_pose_.pose.position.x;
        double dy = lookahead_pose.pose.position.y - current_pose_.pose.position.y;
        double distance = std::hypot(dx, dy);
        
        // 目标方向
        double target_yaw = std::atan2(dy, dx);
        double robot_yaw = tf::getYaw(current_pose_.pose.orientation);
        double angle_error = angles::shortest_angular_distance(robot_yaw, target_yaw);
        
        // 使用曲率控制（更平滑）
        double curvature = calculateCurvature(current_pose_, lookahead_pose);
        
        // 计算线速度（距离越近速度越慢）
        linear_vel = linear_kp_ * distance;
        
        // 计算角速度（结合角度误差和曲率）
        angular_vel = angular_kp_ * angle_error + curvature * linear_vel;
        
        // 接近目标时减速
        if (distance_to_goal < max_linear_vel_ * 2) {
            linear_vel *= (distance_to_goal / (max_linear_vel_ * 2));
        }
        
        // 大角度误差时优先旋转
        if (std::abs(angle_error) > M_PI/4) {
            linear_vel *= 0.3;
        }
        
        // 应用加速度限制
        applyAccelerationLimits(linear_vel, angular_vel);
        
        // 限制最大速度
        limitVelocity(linear_vel, angular_vel);
        
        // 发布控制命令
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;
        
        cmd_vel_pub_.publish(cmd_vel);
        
        ROS_INFO_THROTTLE(1, "Control: lin=%.2f, ang=%.2f, dist_to_goal=%.2f", 
                         linear_vel, angular_vel, distance_to_goal);
    }
    
    void stopRobot() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_vel);
        
        current_linear_vel_ = 0.0;
        current_angular_vel_ = 0.0;
    }
    
    void run() {
        ros::Rate rate(20); // 20Hz
        
        while (ros::ok()) {
            controlLoop();
            ros::spinOnce();
            rate.sleep();
        }
        
        // 退出时停止机器人
        stopRobot();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "smooth_path_follower");
    
    SmoothPathFollower follower;
    follower.run();
    
    return 0;
}