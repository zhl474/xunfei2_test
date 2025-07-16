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
        if (req.lidar_process_start!=-1 && req.lidar_process_start!=0){//这部分是找板用的雷达处理代码，后面是巡线避障的雷达处理代码
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
                ROS_INFO("距离%f",disdance[effective_point/2]);//中位数
                resp.lidar_results.push_back(disdance[effective_point/2]);
                return true;
            }

            if(req.lidar_process_start==2){//到达板前，雷达对准
                int effective_point = 0;
                for (int i=158;i<=178;i++) {// 将雷达数据转化为xy坐标系
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
                    slope.push_back((result[i+1][0]-result[i][0])/(result[i+1][1]-result[i][1])*-1);//这里是x/y，免得斜率变成无穷大了
                }
                std::sort(slope.begin(), slope.end());
                ROS_INFO("板子斜率%f",slope[effective_point/2]);
                resp.lidar_results.push_back(slope[effective_point/2]);
                return true;
            }
            return true;
        }
    //---------------------------新增的模式：为避障获取精确前方距离和角度---------------------
        if(req.lidar_process_start == 0){ 
            if (lasar_scan_.ranges.empty()) { // 检查是否有雷达数据
                ROS_INFO("无雷达数据");
                resp.lidar_results.push_back(-1); // 返回-1表示无数据
                return true;
            }
            float min_dist = std::numeric_limits<float>::infinity();
            int min_index = -1;
    
            // 根据要求，检查索引150到186的范围
            for(int i = 120; i <= 216; i++){
                // 安全检查，防止索引越界
                if(i >= lasar_scan_.ranges.size()) break; 
        
                float current_range = lasar_scan_.ranges[i];
        
                // 忽略无效数据
                if(std::isinf(current_range) || std::isnan(current_range) || current_range <= 0.0f) continue;
        
                // 寻找最小值
                if(current_range < min_dist){
                    min_dist = current_range;
                    min_index = i;
                }
            }

            if(min_index != -1){ // 如果找到了有效点
                // 使用LaserScan消息中的元数据计算精确角度
                float angle = lasar_scan_.angle_min + min_index * lasar_scan_.angle_increment;
        
                // 返回最小距离和对应的精确角度
                resp.lidar_results.push_back(min_dist);
                resp.lidar_results.push_back(angle);
                ROS_INFO("找到有效点");
            } else {
                // 如果在该范围内没有找到有效点，返回-1
                resp.lidar_results.push_back(-1); 
                ROS_INFO("未找到有效点");
            }
            return true;
        }
        
        // else if(req.lidar_process_start==-1){//视觉巡线区的雷达处理代码
        //     int effective_point = 0;
        //     double average_x = 0;
        //     double average_y = 0;
        //     std::vector<cv::Point2f> points;//准备拟合直线
        //     std::vector<float> distance;
        //     bool flag = 1;
        //     int count = 168;
        //     while(flag){
        //         count++;
        //         if (count>332) break;
        //         if(std::isinf(ranges_[count]) || ranges_[count] == 0.0f) {
        //             continue;
        //         }
        //         if (fabs(ranges_[count+1]-ranges_[count])>0.2) break;
        //         if (ranges_[count] > 0.6) continue;
        //         theta = count * angle_step;
        //         cv::Point2f pt(ranges_[count] * cos(theta) * -1, ranges_[count] * sin(theta) * -1);//因为180度才是正前方，差了一个π所以*-1
        //         points.push_back(pt);
        //         effective_point++;
        //         average_x += ranges_[count] * cos(theta)*-1;
        //         average_y += ranges_[count] * sin(theta)*-1;
        //         distance.push_back(ranges_[count]);
        //     }
        //     if (effective_point==0){
        //         resp.lidar_results.push_back(-1);
        //         return true;
        //     }
        //     count = 168;
        //     while(flag){
        //         count--;
        //         if (count==3) break;
        //         if(std::isinf(ranges_[count]) || ranges_[count] == 0.0f) {
        //             continue;
        //         }
        //         if (fabs(ranges_[count+1]-ranges_[count])>0.2) break;
        //         if (ranges_[count] > 0.6) continue;
        //         theta = count * angle_step;
        //         cv::Point2f pt(ranges_[count] * cos(theta) * -1, ranges_[count] * sin(theta) * -1);//因为180度才是正前方，差了一个π所以*-1
        //         points.push_back(pt);
        //         effective_point++;
        //         average_x += ranges_[count] * cos(theta)*-1;
        //         average_y += ranges_[count] * sin(theta)*-1;
        //         distance.push_back(ranges_[count]);
        //     }
        //     std::sort(distance.begin(), distance.end());
        //     ROS_INFO("最小距离%f",distance[0]);
        //     ROS_INFO("有效点数%d",effective_point);
        //     if (effective_point > 5 && distance[0] < 0.45){ //满足条件就说明前方有障碍物
        //         // waitForContinue();
        //         cv::Vec4f lineParams;
        //         cv::fitLine(points, lineParams, cv::DIST_L2, 0, 0.01, 0.01);
        //         resp.lidar_results.push_back(distance[0]);//最短距离
        //         resp.lidar_results.push_back(average_x/effective_point);//中点x坐标
        //         resp.lidar_results.push_back(average_y/effective_point);//中点y坐标
        //         resp.lidar_results.push_back(lineParams[0]/lineParams[1]);//板子斜率
        //         ROS_INFO("已返回障碍物斜率%f",lineParams[0]/lineParams[1]);
        //         return true;
        //     }
        //     else {//从else回来的第一项=-1就是没有障碍物
        //         resp.lidar_results.push_back(-1);
        //         return true;
        //     }
        // }
        else if(req.lidar_process_start==-2){//视觉巡线区的雷达处理代码第二版，考虑到雷达丢数据
            int effective_point = 0;
            double average_x = 0;
            double average_y = 0;
            std::vector<cv::Point2f> points;//准备拟合直线
            std::vector<float> distance;
            bool flag = 1;
            int count = 168;
            int failed_count = 0;
            while(flag){
                count++;
                if (count>332) break;
                if(std::isinf(ranges_[count]) || ranges_[count] == 0.0f) {
                    continue;
                }
                if (fabs(ranges_[count+1]-ranges_[count])>0.2) failed_count++;
                if (failed_count > 6) break;
                theta = count * angle_step;
                cv::Point2f pt(ranges_[count] * cos(theta) * -1, ranges_[count] * sin(theta) * -1);//因为180度才是正前方，差了一个π所以*-1
                points.push_back(pt);
                effective_point++;
                average_x += ranges_[count] * cos(theta)*-1;
                average_y += ranges_[count] * sin(theta)*-1;
                distance.push_back(ranges_[count]);
            }
            if (effective_point==0){
                resp.lidar_results.push_back(-1);
                return true;
            }
            failed_count = 0;
            count = 168;
            while(flag){
                count--;
                if (count==3) break;
                if(std::isinf(ranges_[count]) || ranges_[count] == 0.0f) {
                    continue;
                }
                if (fabs(ranges_[count+1]-ranges_[count])>0.2) failed_count++;
                if (failed_count > 6) break;
                theta = count * angle_step;
                cv::Point2f pt(ranges_[count] * cos(theta) * -1, ranges_[count] * sin(theta) * -1);//因为180度才是正前方，差了一个π所以*-1
                points.push_back(pt);
                effective_point++;
                average_x += ranges_[count] * cos(theta)*-1;
                average_y += ranges_[count] * sin(theta)*-1;
                distance.push_back(ranges_[count]);
            }
            std::sort(distance.begin(), distance.end());//
            ROS_INFO("最小距离%f",distance[0]);
            ROS_INFO("有效点数%d",effective_point);
            if (effective_point > 5 && distance[0] < 0.45){ //满足条件就说明前方有障碍物
                // waitForContinue();
                cv::Vec4f lineParams;
                cv::fitLine(points, lineParams, cv::DIST_L2, 0, 0.01, 0.01);
                resp.lidar_results.push_back(distance[0]);//最短距离
                resp.lidar_results.push_back(average_x/effective_point);//中点x坐标
                resp.lidar_results.push_back(average_y/effective_point);//中点y坐标
                resp.lidar_results.push_back(lineParams[0]/lineParams[1]);//板子斜率
                ROS_INFO("已返回障碍物斜率%f",lineParams[0]/lineParams[1]);
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