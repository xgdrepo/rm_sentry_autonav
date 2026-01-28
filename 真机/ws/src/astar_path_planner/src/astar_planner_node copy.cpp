#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>

struct Node {
    int x, y;
    double g, h, f;
    Node* parent;
    
    Node(int _x, int _y) : x(_x), y(_y), g(0), h(0), f(0), parent(nullptr) {}
    
    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

class AStarPlanner {
private:
    ros::NodeHandle nh_;
    ros::Subscriber amcl_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher path_pub_;
    
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped goal_pose_;
    nav_msgs::OccupancyGrid::ConstPtr map_;
    
    bool pose_received_;
    bool goal_received_;
    bool map_received_;
    
    double robot_radius_;
    double resolution_;
    int width_, height_;
    
    // 参数变量
    std::string amcl_topic_;
    std::string goal_topic_;
    std::string map_topic_;
    std::string path_topic_;
    double pose_update_rate_;
    double planning_rate_;
    
    std::mutex pose_mutex_;
    std::mutex goal_mutex_;
    std::mutex map_mutex_;
    
    ros::Timer planning_timer_;
    ros::Timer pose_timer_;
    
public:
    AStarPlanner() : 
        pose_received_(false),
        goal_received_(false),
        map_received_(false),
        robot_radius_(0.5),
        resolution_(0.05),
        pose_update_rate_(10.0),
        planning_rate_(2.0) {
        
        // 从参数服务器读取参数
        nh_.param("robot_radius", robot_radius_, 0.5);
        nh_.param("amcl_topic", amcl_topic_, std::string("/amcl_pose"));
        nh_.param("goal_topic", goal_topic_, std::string("/goal"));
        nh_.param("map_topic", map_topic_, std::string("/map"));
        nh_.param("path_topic", path_topic_, std::string("/path"));
        nh_.param("pose_update_rate", pose_update_rate_, 10.0);
        nh_.param("planning_rate", planning_rate_, 2.0);
        
        ROS_INFO("AStar Planner Parameters:");
        ROS_INFO("  amcl_topic: %s", amcl_topic_.c_str());
        ROS_INFO("  goal_topic: %s", goal_topic_.c_str());
        ROS_INFO("  map_topic: %s", map_topic_.c_str());
        ROS_INFO("  path_topic: %s", path_topic_.c_str());
        ROS_INFO("  pose_update_rate: %.1f Hz", pose_update_rate_);
        ROS_INFO("  planning_rate: %.1f Hz", planning_rate_);
        ROS_INFO("  robot_radius: %.2f m", robot_radius_);
        
        // 订阅器
        amcl_sub_ = nh_.subscribe(amcl_topic_, 10, &AStarPlanner::amclCallback, this);
        goal_sub_ = nh_.subscribe(goal_topic_, 10, &AStarPlanner::goalCallback, this);
        map_sub_ = nh_.subscribe(map_topic_, 1, &AStarPlanner::mapCallback, this);
        
        // 发布器
        path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1);
        
        // 计算定时器周期
        double pose_update_period = 1.0 / pose_update_rate_;
        double planning_period = 1.0 / planning_rate_;
        
        // 定时器：位置数据更新
        pose_timer_ = nh_.createTimer(ros::Duration(pose_update_period), 
            boost::bind(&AStarPlanner::poseUpdate, this));
        
        // 定时器：路径规划
        planning_timer_ = nh_.createTimer(ros::Duration(planning_period),
            boost::bind(&AStarPlanner::planningCallback, this));
    }
    
    void poseUpdate() {
        // 保持位置更新频率
        std::lock_guard<std::mutex> lock(pose_mutex_);
        // 这里可以添加额外的处理
    }
    
    void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        current_pose_.header = msg->header;
        current_pose_.pose = msg->pose.pose;
        pose_received_ = true;
    }
    
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        goal_pose_ = *msg;
        goal_received_ = true;
    }
    
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_ = msg;
        resolution_ = msg->info.resolution;
        width_ = msg->info.width;
        height_ = msg->info.height;
        map_received_ = true;
    }
    
    bool worldToMap(double wx, double wy, int& mx, int& my) {
        if (!map_received_) return false;
        
        mx = (wx - map_->info.origin.position.x) / resolution_;
        my = (wy - map_->info.origin.position.y) / resolution_;
        
        return (mx >= 0 && mx < width_ && my >= 0 && my < height_);
    }
    
    void mapToWorld(int mx, int my, double& wx, double& wy) {
        if (!map_received_) return;
        
        wx = map_->info.origin.position.x + mx * resolution_;
        wy = map_->info.origin.position.y + my * resolution_;
    }
    
    bool isCellFree(int x, int y) {
        if (!map_received_) return false;
        if (x < 0 || x >= width_ || y < 0 || y >= height_) return false;
        
        // 检查机器人半径内的碰撞
        int check_radius = std::ceil(robot_radius_ / resolution_);
        
        for (int dx = -check_radius; dx <= check_radius; ++dx) {
            for (int dy = -check_radius; dy <= check_radius; ++dy) {
                int nx = x + dx;
                int ny = y + dy;
                
                if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
                    return false;
                }
                
                int index = ny * width_ + nx;
                if (map_->data[index] > 50 || map_->data[index] == -1) {
                    return false;
                }
            }
        }
        
        return true;
    }
    
    double heuristic(int x1, int y1, int x2, int y2) {
        // 欧几里得距离
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    }
    
    std::vector<Node*> getNeighbors(Node* node) {
        std::vector<Node*> neighbors;
        
        // 8方向移动
        int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1};
        int dy[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        
        for (int i = 0; i < 8; ++i) {
            int nx = node->x + dx[i];
            int ny = node->y + dy[i];
            
            if (isCellFree(nx, ny)) {
                neighbors.push_back(new Node(nx, ny));
            }
        }
        
        return neighbors;
    }
    
    std::vector<geometry_msgs::PoseStamped> findPath(int start_x, int start_y, int goal_x, int goal_y) {
        std::vector<geometry_msgs::PoseStamped> path;
        
        if (!isCellFree(start_x, start_y) || !isCellFree(goal_x, goal_y)) {
            ROS_WARN("Start or goal position is not free!");
            return path;
        }
        
        auto cmp = [](Node* a, Node* b) { return a->f > b->f; };
        std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open_list(cmp);
        std::vector<std::vector<bool>> closed_list(width_, std::vector<bool>(height_, false));
        
        Node* start_node = new Node(start_x, start_y);
        start_node->h = heuristic(start_x, start_y, goal_x, goal_y);
        start_node->f = start_node->h;
        open_list.push(start_node);
        
        while (!open_list.empty()) {
            Node* current = open_list.top();
            open_list.pop();
            
            if (closed_list[current->x][current->y]) {
                delete current;
                continue;
            }
            
            closed_list[current->x][current->y] = true;
            
            // 到达目标点
            if (current->x == goal_x && current->y == goal_y) {
                // 回溯路径
                while (current != nullptr) {
                    geometry_msgs::PoseStamped pose;
                    mapToWorld(current->x, current->y, pose.pose.position.x, pose.pose.position.y);
                    pose.pose.orientation.w = 1.0;
                    path.push_back(pose);
                    current = current->parent;
                }
                std::reverse(path.begin(), path.end());
                break;
            }
            
            std::vector<Node*> neighbors = getNeighbors(current);
            
            for (Node* neighbor : neighbors) {
                if (closed_list[neighbor->x][neighbor->y]) {
                    delete neighbor;
                    continue;
                }
                
                double tentative_g = current->g + 
                    std::sqrt(std::pow(neighbor->x - current->x, 2) + 
                             std::pow(neighbor->y - current->y, 2));
                
                neighbor->g = tentative_g;
                neighbor->h = heuristic(neighbor->x, neighbor->y, goal_x, goal_y);
                neighbor->f = neighbor->g + neighbor->h;
                neighbor->parent = current;
                
                open_list.push(neighbor);
            }
        }
        
        // 清理内存
        while (!open_list.empty()) {
            delete open_list.top();
            open_list.pop();
        }
        
        return path;
    }
    
    void planningCallback() {
        if (!pose_received_ || !goal_received_ || !map_received_) {
            return;
        }
        
        std::lock_guard<std::mutex> lock1(pose_mutex_);
        std::lock_guard<std::mutex> lock2(goal_mutex_);
        std::lock_guard<std::mutex> lock3(map_mutex_);
        
        int start_x, start_y, goal_x, goal_y;
        
        if (!worldToMap(current_pose_.pose.position.x, 
                       current_pose_.pose.position.y, 
                       start_x, start_y) ||
            !worldToMap(goal_pose_.pose.position.x,
                       goal_pose_.pose.position.y,
                       goal_x, goal_y)) {
            ROS_WARN("Failed to convert world coordinates to map coordinates!");
            return;
        }
        
        std::vector<geometry_msgs::PoseStamped> path_poses = findPath(start_x, start_y, goal_x, goal_y);
        
        if (!path_poses.empty()) {
            nav_msgs::Path path;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = "map";
            path.poses = path_poses;
            
            path_pub_.publish(path);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner_node");
    
    AStarPlanner planner;
    
    ros::spin();
    
    return 0;
}