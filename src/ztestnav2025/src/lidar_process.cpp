#include <ros/ros.h>  
#include <sensor_msgs/LaserScan.h>  
#include "ztestnav2025/lidar_process.h"

#include <vector>  
#include <cmath>  

class LidarProcessor {
private:
    ros::NodeHandle nh_; 
    ros::Subscriber sub_;
    ros::ServiceServer server;
    sensor_msgs::LaserScan lasar_scan_;
    std::vector<float> ranges_;
    int num_points_ = 337;
    float angle_step = M_PI * 2.0 / (num_points_ - 1);

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        lasar_scan_ = *scan_msg;  // 更新最新扫描数据
    }

    bool lidar_process(ztestnav2025::lidar_process::Request& req,ztestnav2025::lidar_process::Response& resp){
        ranges_ = lasar_scan_.ranges;
        num_points_ = ranges_.size();
        std::vector<std::vector<float>> result;
        float theta = 0;
        for (size_t i = 0; i < num_points_; ++i) {
            if (std::isinf(ranges_[i]) || ranges_[i] == 0.0f) {
                result.push_back({0.0f, 0.0f});
                continue;
            }

            theta = i * angle_step;

            result.push_back({
                ranges_[i] * cos(theta), // x坐标
                ranges_[i] * sin(theta)  // y坐标
            });
        }
        for (size_t i = 0; i < num_points_; ++i) {
            for (size_t j = 0; j < 2; ++j) 
                std::cout << result[i][j] << " ";
            std::cout << "\n";
        }
        return true;
    }
    
public:
    LidarProcessor() : nh_("~") {
        // 订阅雷达数据（队列大小10）
        sub_ = nh_.subscribe("/scan", 10, &LidarProcessor::scanCallback, this);
        server = nh_.advertiseService("lidar_process", &LidarProcessor::lidar_process, this);
        ROS_INFO("雷达初始化");
    }

    // 实时查看数据（新增方法）
    void printCurrentData() {
        if (lasar_scan_.ranges.empty()) {
            ROS_WARN("No scan data available");
            return;
        }
        // 打印最新数据摘要
        ROS_INFO("Latest scan: min=%.2fm, max=%.2fm, angle=%.2frad", 
                lasar_scan_.range_min, 
                lasar_scan_.range_max,
                lasar_scan_.angle_max - lasar_scan_.angle_min);
    }
};

int main(int argc, char *argv[]) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "lidar_process");
    
    // 初始化存储对象
    LidarProcessor lidar_processor;
    
    ros::spin();
    return 0;
}