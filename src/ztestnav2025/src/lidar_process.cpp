#include <ros/ros.h>  
#include <sensor_msgs/LaserScan.h>  
#include "ztestnav2025/lidar_process.h"
#include "ztestnav2025/traffic_light.h"

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
        if (req.lidar_process_start!=-1){//这部分是找板用的雷达处理代码，后面是巡线避障的雷达处理代码
            if(req.lidar_process_start==1){//视觉发现板子，雷达查看正前方数据
                int effective_point = 0;
                std::vector<float> disdance;
                for(int i=158;i<=178;i++){//只看正前方的点
                    if(std::isinf(ranges_[i]) || ranges_[i] == 0.0f) continue;
                    effective_point++;
                    disdance.push_back(ranges_[i]);
                }
                ROS_INFO("有效点数%d",effective_point);
                std::sort(disdance.begin(), disdance.end());
                ROS_INFO("平均距离%f",disdance[effective_point/2]);
                resp.lidar_results.push_back(disdance[effective_point/2]);
                return true;
            }
            //-----------此处为利用数学方法改进找板导航逻辑部分，暂用注释保存-----------------
            //// 当请求为1时，是视觉对准后，雷达查看正前方数据
        //     if (req.lidar_process_start == 1) {
        //         int effective_point = 0;
        //         std::vector<float> disdance;
                
        //         //  根据标定，正前方索引为118。我们取116-120共5个点作为一个小的窗口来增加鲁棒性。
        //         // 确保索引不越界
        //         int start_index = 116;
        //         int end_index = 120;
        //         if (start_index < 0 || end_index >= num_points_) {
        //             ROS_ERROR("雷达索引范围 [%d, %d] 超出雷达数据大小 %d。", start_index, end_index, num_points_);
        //             return false;
        //         }

        //         ROS_INFO("正在检查索引 %d 到 %d 范围内的雷达数据...", start_index, end_index);
        //         for (int i = start_index; i <= end_index; i++) {
        //             // 过滤无效点
        //             if (std::isinf(ranges_[i]) || std::isnan(ranges_[i]) || ranges_[i] <= 0.0f) continue;
        //             effective_point++;
        //             disdance.push_back(ranges_[i]);
        //         }
        //         ROS_INFO("雷达正前方检测到 %d 个有效点", effective_point);

        //         // [鲁棒性检查] 如果有效点太少，认为测量不可靠
        //         if (effective_point < 3) { // 在5个点的窗口中，至少有3个点是有效的
        //             ROS_WARN("正前方有效雷达点过少，距离测量可能不准确。");
        //             resp.lidar_results.push_back(-1.0); // 返回一个无效值
        //             return true;
        //         }

        //         // 使用中值滤波，对离群值有很好的鲁棒性
        //         std::sort(disdance.begin(), disdance.end());
        //         double median_distance = disdance[effective_point / 2];
        //         ROS_INFO("距离中值为: %.3f 米", median_distance);
        //         resp.lidar_results.push_back(median_distance);
        //         return true;
        //     }
        //--------------------------分界线--------------------------
            if(req.lidar_process_start==2){//到达板前，雷达对准
                int effective_point = 0;
                for (int i=138;i<=198;i++) {// 将雷达数据转化为xy坐标系
                    if (std::isinf(ranges_[i]) || ranges_[i] == 0.0f) {
                        continue;
                    }
                    theta = i * angle_step;
                    effective_point++;
                    result.push_back({
                        ranges_[i] * cos(theta) * -1, // x坐标与小车同向
                        ranges_[i] * sin(theta) * -1  // y坐标朝左
                    });
                }
                std::vector<double> slope;
                for (int i=0;i<effective_point-1;i++){
                    slope.push_back((result[i+1][0]-result[i][0])-(result[i+1][1]-result[i][1]));//这里是x/y，免得斜率变成无穷大了
                }
                std::sort(slope.begin(), slope.end());
                ROS_INFO("板子斜率%f",slope[effective_point/2]);
                resp.lidar_results.push_back(slope[effective_point/2]);
                return true;
            }
            return true;
        }
        
      else if(req.lidar_process_start==-1){//视觉巡线区的雷达处理代码
            int effective_point = 0;
            double average_distance = 0;
            std::vector<cv::Point2f> points;//准备拟合直线
            std::vector<float> distance;
            bool flag = 1;
            int count = 168;
            while(flag){
                count++;
                if (count>332) break;
                if(std::isinf(ranges_[count]) || ranges_[count] == 0.0f) {
                    continue;
                }
                if (fabs(ranges_[count+1]-ranges_[count])>0.2) break;
                if (ranges_[count] > 0.6) continue;
                theta = count * angle_step;
                cv::Point2f pt(ranges_[count] * cos(theta) * -1, ranges_[count] * sin(theta) * -1);
                points.push_back(pt);
                effective_point++;
                // average_distance += ranges_[count];
                distance.push_back(ranges_[count]);
            }
            if (effective_point==0){
                resp.lidar_results.push_back(-1);
                return true;
            }
            count = 168;
            while(flag){
                count--;
                if (count==3) break;
                if(std::isinf(ranges_[count]) || ranges_[count] == 0.0f) {
                    continue;
                }
                if (fabs(ranges_[count+1]-ranges_[count])>0.2) break;
                if (ranges_[count] > 0.6) continue;
                theta = count * angle_step;
                cv::Point2f pt(ranges_[count] * cos(theta) * -1, ranges_[count] * sin(theta) * -1);
                points.push_back(pt);
                effective_point++;
                // average_distance += ranges_[count];
                distance.push_back(ranges_[count]);
            }
            std::sort(distance.begin(), distance.end());
            ROS_INFO("最小距离%f",distance[0]);
            ROS_INFO("有效点数%d",effective_point);
            if (effective_point > 5 && distance[0] < 0.45){ //满足条件就说明前方有障碍物
                // waitForContinue();
                cv::Vec4f lineParams;
                cv::fitLine(points, lineParams, cv::DIST_L2, 0, 0.01, 0.01);
                resp.lidar_results.push_back(distance[0]);
                resp.lidar_results.push_back(-1);
                resp.lidar_results.push_back(lineParams[0]);
                resp.lidar_results.push_back(lineParams[1]);
                ROS_INFO("已返回障碍物斜率%f,%f",lineParams[0],lineParams[1]);
                return true;
            }
            else {//从else回来的第一项=-1就是没有障碍物
                resp.lidar_results.push_back(-1);
                return true;
            }
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