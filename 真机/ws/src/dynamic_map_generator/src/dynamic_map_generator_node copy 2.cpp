#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>
#include <mutex>

class DynamicMapGenerator {
public:
    DynamicMapGenerator() : 
        tf_listener_(tf_buffer_),
        map_resolution_(0.05),
        map_width_(0),
        map_height_(0),
        map_origin_x_(0.0),
        map_origin_y_(0.0),
        has_static_map_(false) {
        
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        
        // 参数
        private_nh.param("map_resolution", map_resolution_, 0.05);
        private_nh.param("map_frame", map_frame_, std::string("map"));
        private_nh.param("odom_frame", odom_frame_, std::string("odom"));
        private_nh.param("base_frame", base_frame_, std::string("base_link"));
        private_nh.param("lidar_frame", lidar_frame_, std::string("lidar_link"));
        private_nh.param("static_map_topic", static_map_topic_, std::string("/map"));
        
        // 初始化地图参数
        private_nh.param("map_width", map_width_, 400);  // 20米宽
        private_nh.param("map_height", map_height_, 400); // 20米高
        private_nh.param("map_origin_x", map_origin_x_, -10.0); // 原点在中心
        private_nh.param("map_origin_y", map_origin_y_, -10.0);
        
        // 发布者和订阅者
        scan_sub_ = nh.subscribe("/scan", 1, &DynamicMapGenerator::scanCallback, this);
        static_map_sub_ = nh.subscribe(static_map_topic_, 1, &DynamicMapGenerator::staticMapCallback, this);
        map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/map1", 1);
        
        // 初始化地图
        initializeMap();
        
        // 10Hz发布频率
        ros::Rate rate(10);
        while (ros::ok()) {
            publishMap();
            ros::spinOnce();
            rate.sleep();
        }
    }
    
private:
    void initializeMap() {
        // 重置地图数据
        dynamic_map_data_.clear();
        dynamic_map_data_.resize(map_width_ * map_height_, -1); // -1表示未知
        
        // 初始化地图消息
        map_msg_.header.frame_id = map_frame_;
        map_msg_.info.resolution = map_resolution_;
        map_msg_.info.width = map_width_;
        map_msg_.info.height = map_height_;
        map_msg_.info.origin.position.x = map_origin_x_;
        map_msg_.info.origin.position.y = map_origin_y_;
        map_msg_.info.origin.position.z = 0.0;
        map_msg_.info.origin.orientation.w = 1.0;
        map_msg_.info.origin.orientation.x = 0.0;
        map_msg_.info.origin.orientation.y = 0.0;
        map_msg_.info.origin.orientation.z = 0.0;
        map_msg_.data.resize(map_width_ * map_height_);
    }
    
    void staticMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map_msg) {
        std::lock_guard<std::mutex> lock(static_map_mutex_);
        
        // 保存静态地图
        static_map_ = *map_msg;
        has_static_map_ = true;
        
        ROS_INFO("Received static map: %dx%d, resolution: %f", 
                 map_msg->info.width, map_msg->info.height, map_msg->info.resolution);
    }
    
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        try {
            // 重置动态地图为全未知（白色）
            std::fill(dynamic_map_data_.begin(), dynamic_map_data_.end(), -1);
            
            // 获取从lidar_link到map的变换
            geometry_msgs::TransformStamped transform;
            try {
                // 尝试直接获取lidar_link到map的变换
                transform = tf_buffer_.lookupTransform(
                    map_frame_, 
                    scan_msg->header.frame_id.empty() ? lidar_frame_ : scan_msg->header.frame_id,
                    scan_msg->header.stamp,
                    ros::Duration(0.1)
                );
            } catch (tf2::LookupException& e) {
                // 如果失败，尝试通过base_link中转
                ros::Time now = ros::Time::now();
                tf_buffer_.canTransform(map_frame_, base_frame_, now, ros::Duration(0.1));
                tf_buffer_.canTransform(base_frame_, scan_msg->header.frame_id.empty() ? lidar_frame_ : scan_msg->header.frame_id, now, ros::Duration(0.1));
                
                auto transform1 = tf_buffer_.lookupTransform(
                    map_frame_,
                    base_frame_,
                    scan_msg->header.stamp
                );
                
                auto transform2 = tf_buffer_.lookupTransform(
                    base_frame_,
                    scan_msg->header.frame_id.empty() ? lidar_frame_ : scan_msg->header.frame_id,
                    scan_msg->header.stamp
                );
                
                // 合并变换
                tf2::Transform tf1, tf2;
                tf2::fromMsg(transform1.transform, tf1);
                tf2::fromMsg(transform2.transform, tf2);
                tf2::Transform tf_result = tf1 * tf2;
                
                transform.transform = tf2::toMsg(tf_result);
                transform.header.stamp = scan_msg->header.stamp;
                transform.header.frame_id = map_frame_;
                transform.child_frame_id = scan_msg->header.frame_id.empty() ? lidar_frame_ : scan_msg->header.frame_id;
            }
            
            // 处理扫描数据
            processScan(scan_msg, transform);
            
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1.0, "TF exception: %s", ex.what());
        }
    }
    
    void processScan(const sensor_msgs::LaserScan::ConstPtr& scan, 
                     const geometry_msgs::TransformStamped& transform) {
        
        tf2::Transform tf_map_lidar;
        tf2::fromMsg(transform.transform, tf_map_lidar);
        
        float angle = scan->angle_min;
        
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];
            
            // 过滤无效数据
            if (std::isinf(range) || std::isnan(range) || range < scan->range_min || range > scan->range_max) {
                angle += scan->angle_increment;
                continue;
            }
            
            // 计算激光点在lidar坐标系中的位置
            double x_lidar = range * cos(angle);
            double y_lidar = range * sin(angle);
            
            // 转换到map坐标系
            tf2::Vector3 point_lidar(x_lidar, y_lidar, 0.0);
            tf2::Vector3 point_map = tf_map_lidar * point_lidar;
            
            // 转换到动态地图栅格坐标
            int map_x = static_cast<int>((point_map.x() - map_origin_x_) / map_resolution_);
            int map_y = static_cast<int>((point_map.y() - map_origin_y_) / map_resolution_);
            
            // 检查是否在动态地图范围内
            if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_) {
                int index = map_y * map_width_ + map_x;
                dynamic_map_data_[index] = 100; // 100表示黑色（占用）
            }
            
            angle += scan->angle_increment;
        }
    }
    
    void publishMap() {
        std::lock_guard<std::mutex> lock(static_map_mutex_);
        
        // 创建新的地图数据，先复制动态地图
        std::vector<int8_t> final_map_data = dynamic_map_data_;
        
        // 如果有静态地图，进行叠加
        if (has_static_map_) {
            overlayStaticMap(final_map_data);
        }
        
        // 发布地图
        map_msg_.header.stamp = ros::Time::now();
        
        // 将最终数据复制到地图消息
        for (size_t i = 0; i < final_map_data.size(); ++i) {
            map_msg_.data[i] = final_map_data[i];
        }
        
        map_pub_.publish(map_msg_);
    }
    
    void overlayStaticMap(std::vector<int8_t>& dynamic_map) {
        const nav_msgs::OccupancyGrid& static_map = static_map_;
        
        // 计算静态地图到动态地图的坐标转换
        double static_resolution = static_map.info.resolution;
        double static_origin_x = static_map.info.origin.position.x;
        double static_origin_y = static_map.info.origin.position.y;
        
        // 遍历动态地图的每个栅格
        for (int dy = 0; dy < map_height_; ++dy) {
            for (int dx = 0; dx < map_width_; ++dx) {
                // 计算动态地图栅格在世界坐标系中的位置
                double world_x = map_origin_x_ + dx * map_resolution_;
                double world_y = map_origin_y_ + dy * map_resolution_;
                
                // 转换到静态地图栅格坐标
                int sx = static_cast<int>((world_x - static_origin_x) / static_resolution);
                int sy = static_cast<int>((world_y - static_origin_y) / static_resolution);
                
                // 检查是否在静态地图范围内
                if (sx >= 0 && sx < static_map.info.width && 
                    sy >= 0 && sy < static_map.info.height) {
                    
                    int static_index = sy * static_map.info.width + sx;
                    int dynamic_index = dy * map_width_ + dx;
                    
                    int8_t static_value = static_map.data[static_index];
                    
                    // 叠加策略：静态地图优先级高于动态地图
                    if (static_value >= 0) { // 静态地图已知区域
                        if (static_value > 0) { // 静态地图有障碍物
                            dynamic_map[dynamic_index] = static_value; // 使用静态地图值
                        } else if (dynamic_map[dynamic_index] == -1) { // 动态地图未知，静态地图空闲
                            dynamic_map[dynamic_index] = 0; // 设为空闲
                        }
                        // 如果动态地图有障碍物(100)，保持障碍物不变
                    }
                    // 如果静态地图是-1（未知），保持动态地图的值不变
                }
            }
        }
    }
    
    // ROS
    ros::Subscriber scan_sub_;
    ros::Subscriber static_map_sub_;
    ros::Publisher map_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // 地图参数
    double map_resolution_;
    int map_width_;
    int map_height_;
    double map_origin_x_;
    double map_origin_y_;
    
    // 坐标系
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string lidar_frame_;
    std::string static_map_topic_;
    
    // 地图数据
    nav_msgs::OccupancyGrid map_msg_;
    std::vector<int8_t> dynamic_map_data_;  // 仅包含当前扫描的动态数据
    
    // 静态地图数据
    nav_msgs::OccupancyGrid static_map_;
    bool has_static_map_;
    std::mutex static_map_mutex_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_map_generator");
    
    try {
        DynamicMapGenerator generator;
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in dynamic_map_generator: %s", e.what());
        return 1;
    }
    
    return 0;
}