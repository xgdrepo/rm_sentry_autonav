#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <deque>

class PathFollower {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher cmd_pub_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    nav_msgs::Path current_path_;
    geometry_msgs::PoseStamped current_pose_;
    bool path_received_;
    bool pose_received_;
    
    double target_distance_;      // 目标点距离
    double control_frequency_;    // 控制频率
    double control_gain_;         // 控制增益
    double stop_threshold_;       // 停止阈值
    double max_linear_speed_;     // 最大线速度
    std::string global_frame_;    // 全局坐标系（如map）
    std::string robot_frame_;     // 机器人坐标系（base_link）
    
    int last_target_idx_;         // 上次目标点的索引
    bool new_path_received_;      // 是否收到新路径
    
    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        if (msg->poses.empty()) return;
        current_path_ = *msg;
        path_received_ = true;
        new_path_received_ = true;  // 标记收到新路径
        last_target_idx_ = 0;       // 重置目标点索引
        // ROS_INFO("Received new path with %zu points", msg->poses.size());
    }
    
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        current_pose_.pose = msg->pose.pose;
        current_pose_.header = msg->header;
        pose_received_ = true;
    }
    
    geometry_msgs::PoseStamped findTargetPoint() {
        if (current_path_.poses.empty()) {
            return current_pose_;
        }
        
        geometry_msgs::PoseStamped target_pose;
        
        // 如果是新路径，从最近点开始
        if (new_path_received_) {
            new_path_received_ = false;
            double min_dist = std::numeric_limits<double>::max();
            
            // 找到距离机器人最近的路点
            for (size_t i = 0; i < current_path_.poses.size(); ++i) {
                double dx = current_path_.poses[i].pose.position.x - current_pose_.pose.position.x;
                double dy = current_path_.poses[i].pose.position.y - current_pose_.pose.position.y;
                double dist = sqrt(dx*dx + dy*dy);
                
                if (dist < min_dist) {
                    min_dist = dist;
                    last_target_idx_ = i;
                }
            }
            // ROS_INFO("Starting from waypoint %d, distance: %.3f", last_target_idx_, min_dist);
        }
        
        // 确保索引在有效范围内
        if (last_target_idx_ >= current_path_.poses.size()) {
            last_target_idx_ = current_path_.poses.size() - 1;
        }
        
        // 从上次目标点开始，沿着路径寻找距离target_distance的目标点
        double accumulated_dist = 0.0;
        for (int i = last_target_idx_; i < (int)current_path_.poses.size() - 1; ++i) {
            double dx = current_path_.poses[i+1].pose.position.x - current_path_.poses[i].pose.position.x;
            double dy = current_path_.poses[i+1].pose.position.y - current_path_.poses[i].pose.position.y;
            double segment_dist = sqrt(dx*dx + dy*dy);
            
            if (accumulated_dist + segment_dist >= target_distance_) {
                double ratio = (target_distance_ - accumulated_dist) / segment_dist;
                target_pose.pose.position.x = current_path_.poses[i].pose.position.x + dx * ratio;
                target_pose.pose.position.y = current_path_.poses[i].pose.position.y + dy * ratio;
                target_pose.pose.position.z = 0.0;
                target_pose.header.frame_id = global_frame_;
                target_pose.header.stamp = ros::Time::now();
                
                // 更新目标点索引，但不要超过当前点
                last_target_idx_ = i;
                
                // ROS_DEBUG("Target point at index %d, accumulated: %.3f", i, accumulated_dist);
                return target_pose;
            }
            accumulated_dist += segment_dist;
        }
        
        // 如果路径不够长，返回最后一个点
        last_target_idx_ = current_path_.poses.size() - 1;
        return current_path_.poses.back();
    }
    
    bool transformPointToRobotFrame(const geometry_msgs::PointStamped& point_in, 
                                  geometry_msgs::PointStamped& point_out) {
        try {
            // 等待变换可用（最多等待0.5秒）
            if (tf_buffer_.canTransform(robot_frame_, point_in.header.frame_id, 
                                       ros::Time(0), ros::Duration(0.5))) {
                point_out = tf_buffer_.transform(point_in, robot_frame_);
                return true;
            } else {
                // ROS_WARN_THROTTLE(1.0, "Cannot transform from %s to %s", 
                //                 point_in.header.frame_id.c_str(), robot_frame_.c_str());
                return false;
            }
        } catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "TF transform error: %s", ex.what());
            return false;
        }
    }
    
    void publishControlCommand() {
        if (!path_received_ || !pose_received_ || current_path_.poses.empty()) {
            return;
        }
        
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        
        // 找到目标点（在全局坐标系中）
        geometry_msgs::PoseStamped target_pose_global = findTargetPoint();
        
        // 创建PointStamped用于TF变换
        geometry_msgs::PointStamped target_point_global;
        target_point_global.header.frame_id = global_frame_;
        target_point_global.header.stamp = ros::Time::now();
        target_point_global.point = target_pose_global.pose.position;
        
        // 转换到机器人坐标系
        geometry_msgs::PointStamped target_point_robot;
        if (transformPointToRobotFrame(target_point_global, target_point_robot)) {
            // 现在target_point_robot是在base_link坐标系中的坐标
            double dx_local = target_point_robot.point.x;
            double dy_local = target_point_robot.point.y;
            
            double distance = sqrt(dx_local*dx_local + dy_local*dy_local);
            
            // 如果距离大于停止阈值，计算控制量
            if (distance > stop_threshold_) {
                // 计算期望速度
                cmd_vel.linear.x = control_gain_ * dx_local;
                cmd_vel.linear.y = control_gain_ * dy_local;
                
                // 速度限制
                double speed = sqrt(cmd_vel.linear.x * cmd_vel.linear.x + cmd_vel.linear.y * cmd_vel.linear.y);
                if (speed > max_linear_speed_) {
                    double scale = max_linear_speed_ / speed;
                    cmd_vel.linear.x *= scale;
                    cmd_vel.linear.y *= scale;
                }
                
                // 检查是否接近终点
                if (target_point_global.header.frame_id != "") {
                    // 获取路径终点在机器人坐标系中的位置
                    geometry_msgs::PointStamped final_point_global;
                    final_point_global.header.frame_id = global_frame_;
                    final_point_global.header.stamp = ros::Time::now();
                    final_point_global.point = current_path_.poses.back().pose.position;
                    
                    geometry_msgs::PointStamped final_point_robot;
                    if (transformPointToRobotFrame(final_point_global, final_point_robot)) {
                        double final_dist = sqrt(final_point_robot.point.x * final_point_robot.point.x + 
                                                final_point_robot.point.y * final_point_robot.point.y);
                        
                        // 如果已经到达终点，停止
                        if (final_dist < stop_threshold_) {
                            cmd_vel.linear.x = 0.0;
                            cmd_vel.linear.y = 0.0;
                            // ROS_INFO("Reached final destination! Distance: %.3f", final_dist);
                        }
                        // 接近终点，减速
                        else if (final_dist < stop_threshold_ * 3.0) {
                            double scale = std::min(1.0, final_dist / (stop_threshold_ * 3.0));
                            cmd_vel.linear.x *= scale;
                            cmd_vel.linear.y *= scale;
                            // ROS_DEBUG_THROTTLE(0.5, "Approaching final point, slowing down. Distance: %.3f", final_dist);
                        }
                    }
                }
                
                // 调试输出
                // ROS_DEBUG_THROTTLE(0.5, "Target: dx=%.3f, dy=%.3f, dist=%.3f, vel=(%.3f,%.3f)", 
                //                 dx_local, dy_local, distance, cmd_vel.linear.x, cmd_vel.linear.y);
            } else {
                // ROS_DEBUG_THROTTLE(0.5, "Reached target point. Distance: %.3f", distance);
                
                // 如果当前目标点已经到达，检查是否到达终点
                if (last_target_idx_ >= current_path_.poses.size() - 1) {
                    geometry_msgs::PointStamped final_point_global;
                    final_point_global.header.frame_id = global_frame_;
                    final_point_global.header.stamp = ros::Time::now();
                    final_point_global.point = current_path_.poses.back().pose.position;
                    
                    geometry_msgs::PointStamped final_point_robot;
                    if (transformPointToRobotFrame(final_point_global, final_point_robot)) {
                        double final_dist = sqrt(final_point_robot.point.x * final_point_robot.point.x + 
                                                final_point_robot.point.y * final_point_robot.point.y);
                        
                        if (final_dist < stop_threshold_) {
                            // ROS_INFO("Path following completed!");
                        }
                    }
                }
            }
        } else {
            // ROS_WARN_THROTTLE(1.0, "Failed to transform target point to robot frame");
            // 如果TF变换失败，使用原来的方法作为备选
            double robot_yaw = 0.0;
            try {
                tf2::Quaternion q(
                    current_pose_.pose.orientation.x,
                    current_pose_.pose.orientation.y,
                    current_pose_.pose.orientation.z,
                    current_pose_.pose.orientation.w
                );
                tf2::Matrix3x3 m(q);
                double roll, pitch;
                m.getRPY(roll, pitch, robot_yaw);
                
                double dx_global = target_pose_global.pose.position.x - current_pose_.pose.position.x;
                double dy_global = target_pose_global.pose.position.y - current_pose_.pose.position.y;
                
                double dx_local = dx_global * cos(robot_yaw) + dy_global * sin(robot_yaw);
                double dy_local = -dx_global * sin(robot_yaw) + dy_global * cos(robot_yaw);
                
                double distance = sqrt(dx_local*dx_local + dy_local*dy_local);
                
                if (distance > stop_threshold_) {
                    cmd_vel.linear.x = control_gain_ * dx_local;
                    cmd_vel.linear.y = control_gain_ * dy_local;
                    
                    double speed = sqrt(cmd_vel.linear.x * cmd_vel.linear.x + cmd_vel.linear.y * cmd_vel.linear.y);
                    if (speed > max_linear_speed_) {
                        double scale = max_linear_speed_ / speed;
                        cmd_vel.linear.x *= scale;
                        cmd_vel.linear.y *= scale;
                    }
                }
            } catch (...) {
                // ROS_ERROR_THROTTLE(1.0, "Failed to calculate fallback control");
            }
        }
        
        cmd_pub_.publish(cmd_vel);
    }
    
public:
    PathFollower() : 
        private_nh_("~"),  // 私有命名空间，用于读取launch文件参数
        tf_listener_(tf_buffer_),
        path_received_(false),
        pose_received_(false),
        target_distance_(0.1),      // 增加默认值，使控制更平滑
        control_frequency_(20.0),   // 增加控制频率
        control_gain_(0.8),
        stop_threshold_(0.05),
        max_linear_speed_(0.3),
        global_frame_("map"),
        robot_frame_("base_link"),
        last_target_idx_(0),
        new_path_received_(false)
    {
        // 从私有命名空间读取launch文件参数
        private_nh_.param("target_distance", target_distance_, 0.1);
        private_nh_.param("control_frequency", control_frequency_, 20.0);
        private_nh_.param("control_gain", control_gain_, 0.8);
        private_nh_.param("stop_threshold", stop_threshold_, 0.05);
        private_nh_.param("max_linear_speed", max_linear_speed_, 0.3);
        private_nh_.param("global_frame", global_frame_, std::string("map"));
        private_nh_.param("robot_frame", robot_frame_, std::string("base_link"));
        
        // 订阅和发布话题
        path_sub_ = nh_.subscribe("path", 1, &PathFollower::pathCallback, this);
        pose_sub_ = nh_.subscribe("pose", 1, &PathFollower::poseCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel1", 1);
        
        // ROS_INFO("Path Follower initialized with waypoint tracking");
        // ROS_INFO("Parameters: target_distance=%.3fm, control_gain=%.2f", target_distance_, control_gain_);
        // ROS_INFO("Parameters: stop_threshold=%.3fm, max_linear_speed=%.2fm/s", stop_threshold_, max_linear_speed_);
        // ROS_INFO("Frames: global=%s, robot=%s", global_frame_.c_str(), robot_frame_.c_str());
        // ROS_INFO("Control frequency: %.0f Hz", control_frequency_);
    }
    
    void run() {
        // 主控制循环
        ros::Rate rate(control_frequency_);
        while (ros::ok()) {
            publishControlCommand();
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower");
    PathFollower follower;
    follower.run();
    return 0;
}