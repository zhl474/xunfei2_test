#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "ztestnav2025/lidar_process.h" // 服务消息头文件

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

class LidarProcessor {
private:
    ros::NodeHandle nh_; 
    ros::Subscriber sub_;
    ros::ServiceServer server_;
    sensor_msgs::LaserScan latest_scan_;
    bool new_scan_received_;
    
    // 参数配置
    float distance_threshold_;  // 聚类距离阈值(米)
    float min_cluster_size_;    // 最小聚类点数
    float max_cluster_size_;    // 最大聚类点数
    std::vector<float> filter_region_; // 过滤区域 [x_min, x_max, y_min, y_max]

public:
    LidarProcessor() : nh_("~"), new_scan_received_(false) {
        // 加载参数配置
        nh_.param<float>("distance_threshold", distance_threshold_, 0.05);
        nh_.param<float>("min_cluster_size", min_cluster_size_, 10);
        nh_.param<float>("max_cluster_size", max_cluster_size_, 1000);
        
        // 默认过滤区域（可通过参数调整）
        filter_region_.resize(4, 0.0f);
        nh_.param<float>("filter_x_min", filter_region_[0], -0.2);
        nh_.param<float>("filter_x_max", filter_region_[1], 2.7);
        nh_.param<float>("filter_y_min", filter_region_[2], -2.2);
        nh_.param<float>("filter_y_max", filter_region_[3], 0.2);
        
        // 订阅激光雷达数据
        sub_ = nh_.subscribe("/scan", 10, &LidarProcessor::scanCallback, this);
        
        // 注册服务
        server_ = nh_.advertiseService("lidar_process", &LidarProcessor::lidarProcessCallback, this);
        
        ROS_INFO("Lidar Processor Node Initialized");
        ROS_INFO("Parameters:");
        ROS_INFO("Distance Threshold: %.2f m", distance_threshold_);
        ROS_INFO("Filter Region: x=[%.2f, %.2f], y=[%.2f, %.2f]", 
                 filter_region_[0], filter_region_[1], filter_region_[2], filter_region_[3]);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        latest_scan_ = *scan_msg;
        new_scan_received_ = true;
        ROS_INFO("[SCAN] Received %zu points", scan_msg->ranges.size()); // 新增日志
    }

    bool lidarProcessCallback(ztestnav2025::lidar_process::Request& req, 
                             ztestnav2025::lidar_process::Response& resp) {
        if (!new_scan_received_) {
            ROS_WARN("No lidar data available");
            return false;
        }

        if (req.lidar_process_start != 1) {
            ROS_WARN("Lidar process not started (req.lidar_process_start=1 required)");
            return false;
        }

        // 数据预处理
        std::vector<std::pair<float, float>> cartesian_points;
        convertToCartesian(latest_scan_, cartesian_points);
        std::vector<bool> valid_points = applyRegionFilter(cartesian_points);

        // 聚类分析
        std::vector<std::vector<size_t>> clusters;
        performClustering(cartesian_points, valid_points, clusters);

        // 构建响应结果
        std::vector<int32_t> result;
        
        // 第一个元素：是否检测到板子（0=未检测到，1=检测到）
        result.push_back(clusters.size() > 0 ? 1 : 0);
        
        // 第二个元素：板子数量
        result.push_back(clusters.size());
        
        // 后续元素：板子坐标（每个板子用x*100和y*100两个整数表示）
        for (const auto& cluster : clusters) {
            float sum_x = 0.0f, sum_y = 0.0f;
            for (size_t idx : cluster) {
                sum_x += cartesian_points[idx].first;
                sum_y += cartesian_points[idx].second;
            }
            float center_x = sum_x / cluster.size();
            float center_y = sum_y / cluster.size();
            
            // 转换为整数（乘以100取整，根据精度需求调整）
            result.push_back(static_cast<int32_t>(center_x * 100));
            result.push_back(static_cast<int32_t>(center_y * 100));
        }

        resp.lidar_results = result;
        ROS_INFO("Detection result: %s, %zu boards found", 
                 clusters.size() > 0 ? "true" : "false", clusters.size());
        return true;
    }

    void convertToCartesian(const sensor_msgs::LaserScan& scan, 
                           std::vector<std::pair<float, float>>& points) {
        points.clear();
        size_t num_points = scan.ranges.size();
        points.resize(num_points);
        
        for (size_t i = 0; i < num_points; ++i) {
            float range = scan.ranges[i];
            if (std::isinf(range) || std::isnan(range) || range < scan.range_min || range > scan.range_max) {
                points[i] = {0.0f, 0.0f};
                continue;
            }
            
            float angle = scan.angle_min + i * scan.angle_increment;
            points[i] = {range * cos(angle), range * sin(angle)};
        }
    }

    std::vector<bool> applyRegionFilter(const std::vector<std::pair<float, float>>& points) {
        std::vector<bool> mask(points.size(), false);
        int valid_count = 0;
        
        for (size_t i = 0; i < points.size(); ++i) {
            float x = points[i].first;
            float y = points[i].second;
            if (x >= filter_region_[0] && x <= filter_region_[1] &&
                y >= filter_region_[2] && y <= filter_region_[3]) {
                mask[i] = true;
                valid_count++;
            }
        }
        
        ROS_INFO("[FILTER] Valid points: %d / %zu", valid_count, points.size());
        return mask;
    }

    void performClustering(const std::vector<std::pair<float, float>>& points,
                          const std::vector<bool>& mask,
                          std::vector<std::vector<size_t>>& clusters) {
        clusters.clear();
        std::vector<bool> visited(points.size(), false);
        
        for (size_t i = 0; i < points.size(); ++i) {
            if (!mask[i] || visited[i]) continue;
            
            std::vector<size_t> cluster;
            std::vector<size_t> queue = {i};
            visited[i] = true;
            
            while (!queue.empty()) {
                size_t current = queue.back();
                queue.pop_back();
                cluster.push_back(current);
                
                for (size_t j = current + 1; j < points.size(); ++j) {
                    if (!mask[j] || visited[j]) continue;
                    
                    float dx = points[j].first - points[current].first;
                    float dy = points[j].second - points[current].second;
                    float dist = std::sqrt(dx*dx + dy*dy);
                    
                    if (dist < distance_threshold_) {
                        visited[j] = true;
                        queue.push_back(j);
                    }
                }
            }
            
            if (cluster.size() >= min_cluster_size_ && cluster.size() <= max_cluster_size_) {
                clusters.push_back(cluster);
            }
        }
        
        ROS_INFO("[CLUSTER] Found %zu clusters", clusters.size());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_test");
    LidarProcessor processor;
    ros::spin();
    return 0;
}
