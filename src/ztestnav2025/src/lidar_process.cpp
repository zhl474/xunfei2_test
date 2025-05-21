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

    bool lidar_process(ztestnav2025::lidar_process::Request& req,ztestnav2025::lidar_process::Response& resp){//处理雷达数据，获取板子坐标
        ranges_ = lasar_scan_.ranges;
        num_points_ = ranges_.size();
        std::vector<std::vector<float>> result;
        float theta = 0;
        for (size_t i = 0; i < num_points_; ++i) {// 将雷达数据转化为xy坐标系
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
        std::vector<int> mask(num_points_,-1);//筛选掉不要的，以及给障碍板分类
        for (size_t i = 0; i < num_points_; ++i) {//筛选掉不要的，以及给障碍板分类
            if(result[i][0]>2.7 || result[i][0]<-0.2 || result[i][1]>0.2 || result[i][1]<-2.2){//去掉和墙壁重合的点
                mask[i] = 0;
            }
        }
        int number = 0;
        int last_eq_first=0;//第一个点和最后一个可能属于同一个板子
        if(mask[0]==1){
            number=1;//如果第一个点就是障碍物，直接记录
            if(mask[-1]==1) last_eq_first=1;
        }
        double pre_dis = ranges_[0];//如果和上一个点的距离发生了突变，说明不属于同一簇
        for (size_t i = 0; i < num_points_; ++i){//给障碍板分类
            if(mask[i] == -1){
                if(ranges_[i]-pre_dis<0.03 && ranges_[i]-pre_dis>-0.03){
                    mask[i] = number;
                }
                else{
                    number++;
                    mask[i] = number;
                }
                pre_dis = ranges_[i];
            }
        }
        if(last_eq_first==1){
            int last_number = mask[-1];
            for(size_t i = 0; i < num_points_; ++i){
                if(mask[i]==last_number) mask[i] = 1;
            }
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