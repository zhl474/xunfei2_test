#include <ros/ros.h>  
#include <sensor_msgs/LaserScan.h>  
#include "ztestnav2025/lidar_process.h"

#include <vector>  
#include <algorithm>
#include <cmath>  

#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>

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
                ranges_[i] * cos(theta) * -1, // x坐标与小车同向
                ranges_[i] * sin(theta) * -1  // y坐标朝左
            });
        }
        std::vector<int> mask(num_points_,-1);//筛选掉不要的，以及给障碍板分类
        if(req.lidar_process_start==1){
            for (size_t i = 0; i < num_points_; ++i) {//筛选掉不要的，以及给障碍板分类
                if(result[i][0]>2.7 || result[i][0]<-0.2 || result[i][1]>0.45 || result[i][1]<-2.0 || result[i][0] == 0){//去掉和墙壁重合的点
                    mask[i] = 0;
                }
            }
        }
        else if(req.lidar_process_start==2){//小车在中心
            for (size_t i = 0; i < num_points_; ++i) {//筛选掉不要的，以及给障碍板分类
                if(result[i][0]>1.2 || result[i][0]<-1.7 || result[i][1]>1.25 || result[i][1]<-1.25 || result[i][0] == 0){//去掉和墙壁重合的点
                    mask[i] = 0;
                }
            }
        }
        else if(req.lidar_process_start==2){//巡线区避障
            for (size_t i = 0; i < num_points_; ++i) {//筛选掉不要的，以及给障碍板分类
                if(result[i][0]>0.6 || result[i][0]<0 || result[i][1]>0.5 || result[i][1]<-0.5 || result[i][0] == 0){//只看前方
                    mask[i] = 0;
                }
            }
        }
        
        int number = 0;
        int last_eq_first=0;//第一个点和最后一个可能属于同一个板子
        if(mask[0]==-1){
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

        auto max_it = std::max_element(mask.begin(), mask.end());
        const int cluster_count = *max_it;//max的值等于几就说明有几个板
        if (cluster_count!=0){
            std::vector<float> board_position_x(cluster_count,0);
            std::vector<float> board_position_y(cluster_count,0);
            std::vector<int> point_number(cluster_count,0);
            std::vector<std::vector<cv::Point2f>> points(cluster_count);

            for (size_t i = 0; i < num_points_; ++i) {
                if(mask[i] != 0){//计算中心坐标并且将每个板子的点分开准备拟合直线
                    board_position_x[mask[i]-1] += result[i][0];
                    board_position_y[mask[i]-1] += result[i][1];
                    point_number[mask[i]-1]++;
                    cv::Point2f pt(result[i][0], result[i][1]);
                    points[mask[i]-1].push_back(pt);
                }
            }
            //求板子中心
            for (size_t i = 0; i < cluster_count; i++) {
                if (point_number[i] != 0){//如果不是0个
                    board_position_x[i] = board_position_x[i] / point_number[i];
                    board_position_y[i] = board_position_y[i] / point_number[i] * -1;//小车坐标系x轴在前方，y轴在左边，现在x轴在右边，y轴在前方
                }
                if(point_number[i] > 4){
                    cv::Vec4f lineParams;
                    cv::fitLine(points[i], lineParams, cv::DIST_L2, 0, 0.01, 0.01);
                    resp.lidar_results.push_back(board_position_x[i]);
                    resp.lidar_results.push_back(board_position_y[i]);
                    resp.lidar_results.push_back(lineParams[0]);
                    resp.lidar_results.push_back(lineParams[1]);
                    ROS_INFO("已返回板子坐标%zu",i);
                }
                else{
                    ROS_INFO("%zu的点数小于4,只有%d个,舍去",i,point_number[i]);
                }
            }
            
            return true;
        }
        else{
            ROS_INFO("这里没有板子");
            return false;
        }
    }
    
public:
    LidarProcessor() : nh_("~") {
        // 订阅雷达数据（队列大小10
        sub_ = nh_.subscribe("/scan", 10, &LidarProcessor::scanCallback, this);
        server = nh_.advertiseService("lidar_process", &LidarProcessor::lidar_process, this);
        ROS_INFO("雷达初始化");
    }

    // 实时查看数据（新增方法）
    void printCurrentData() {
        // ROS_INFO("打印雷达数据");        //调试信息，打印雷达数据
        // std::cout << "[";
        // for (size_t i = 0; i < num_points_; ++i) {
        //     std::cout << "[";
        //     std::cout << result[i][0] << ",";
        //     std::cout << result[i][1];
        //     std::cout << "]";
        //     std::cout << ",";
        // } 
        // std::cout << "]";
        // std::cout << "\n";
        // for (int j=0;j<mask.size();j++){
        //     std::cout << mask[j] << ",";
        // }
        // std::cout << "\n";

        //获取板子坐标
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