#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include<geometry_msgs/Twist.h>
#include <vector>
#include <numeric>

using namespace cv;

class PIDController {
private:
    double kp = 0.2, ki = 0.0, kd = 0.05;  // 更温和的PID参数
    double integral;
    double error;
    ros::Time prev_time;
    int stop_flag = 0;
    double prev_error;
    double straight_line_threshold = 15.0; // 直线判断阈值(像素)
    
public:
    PIDController(double p, double i, double d) : 
        kp(p), ki(i), kd(d), integral(0), error(0) {}
    
    void reset() {
        integral = 0;
    }
    
    double compute(double error) {
        ros::Time now = ros::Time::now();
        double dt = (now - prev_time).toSec();
        prev_time = now;
        
        if (dt <= 0) dt = 0.01;
        
        // 当误差很小时重置积分项，防止积分饱和
        if (fabs(error) < straight_line_threshold) {
            integral = 0;
        } else {
            integral += error * dt;
        }
        
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        
        // 积分限幅
        integral = std::min(0.5, std::max(-0.5, integral));
        
        // 当检测到近似直线时，减小P项的影响
        double effective_kp = kp;
        if (fabs(error) < straight_line_threshold) {
            effective_kp *= 0.3;  // 直线时减小比例系数
        }
        
        return effective_kp * error + ki * integral + kd * derivative;
    }

    void mid(Mat &follow, const Mat &mask, int &error, int &should_stop) {
        int halfWidth = follow.cols / 2;
        should_stop = 0;
        
        // 使用加权平均误差，近处行权重更大
        double weighted_error_sum = 0;
        double weight_sum = 0;
        int valid_lines = 0;

        for (int y = 200; y >= 0; --y) {
            int left = 0, right = follow.cols - 1;
            
            // 左边界检测
            for (int x = halfWidth - 1; x >= 2; --x) {
                if (mask.at<uchar>(y, x) == 255 && 
                    mask.at<uchar>(y, x-1) == 0 && 
                    mask.at<uchar>(y, x-2) == 0) {
                    left = x;
                    break;
                }
            }//ROS_INFO("左边边界%d",left);
            
            // 右边界检测
            for (int x = halfWidth; x < mask.cols - 2; ++x) {
                if (mask.at<uchar>(y, x) == 255 && 
                    mask.at<uchar>(y, x+1) == 0 && 
                    mask.at<uchar>(y, x+2) == 0) {
                    right = x;
                    break;
                }
            }//ROS_INFO("右边边界%d",right);

            // 停车检测
            // if (y == 245) {
            //     int WhiteCount = countNonZero(mask.row(245));
            //     if (WhiteCount >= 250) {
            //         stop_flag = 1;
            //         should_stop = 1;
            //         ROS_INFO("Stop line detected! White pixels: %d", WhiteCount);
            //     }
            // }
            
            // 计算中线
            if (right > left) {  // 有效检测
                int mid = (left + right) / 2;
                follow.at<uchar>(y, mid) = 255;
                
                // 计算当前行误差并加权累加(近处行权重更大)
                double line_error = halfWidth - mid;
                double weight = (double)(y + 1) / follow.rows;  // y越大(越近)，权重越大
                weighted_error_sum += line_error * weight;
                weight_sum += weight;
                valid_lines++;
            }
        
        }
        // 计算加权平均误差
        error = valid_lines > 0 ? weighted_error_sum / weight_sum : 0;
        ROS_INFO("中线%d",error);
            // 可视化最后的中线位置
            if (valid_lines > 0) {
                int last_mid = halfWidth - error;
                circle(follow, Point(last_mid, follow.rows - 1), 5, Scalar(255), -1);
            }
        
    }
};

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "follower_line_pid");
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Rate loop_rate(20);

    PIDController pid_controller(0.2, 0.0, 0.05);  // 使用更温和的参数
    
    VideoCapture cap("/dev/video0", CAP_V4L2);
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera!");
        return -1;
    }
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    
    Mat frame, gray, processed;
    geometry_msgs::Twist twist;
    
    while(ros::ok()) {
        cap.read(frame);
        if (frame.empty()) continue;
        
        // 图像处理
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        threshold(gray, processed, 160, 255, THRESH_BINARY);
        imshow("frame", frame);
        
        
        int error = 0;
        int should_stop = 0;
        
        // 中线检测
        pid_controller.mid(processed, processed, error, should_stop);
        ROS_INFO("\n误差%d",error);
        
        // 控制逻辑
        if (should_stop) {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            ROS_WARN("Stopping!");
        } else {
            // 动态速度控制 - 误差越大速度越小
            double base_speed = 0.15;
            double speed_reduction = 0.08 * (fabs(error)/320.0);
            twist.linear.x = std::max(0.1, base_speed - speed_reduction);
            
            // PID控制
            twist.angular.z = pid_controller.compute(error);
            
            
            // 限幅
            twist.linear.x = std::min(0.2, std::max(0.1, twist.linear.x));
            twist.angular.z = std::min(0.3, std::max(-0.3, twist.angular.z));  // 减小最大角速度
            
            ROS_DEBUG("Error: %d, Linear: %.2f, Angular: %.2f", 
                     error, twist.linear.x, twist.angular.z);
        }
        ROS_INFO("z速度%",twist.angular.z);
        cmd_pub.publish(twist);
        imshow("Processed", processed);
        waitKey(1);
        loop_rate.sleep();
    }
    
    cap.release();
    return 0;
}

// #include<ros/ros.h>
// #include<opencv2/opencv.hpp>
// #include <opencv2/core.hpp>
// #include <opencv2/videoio.hpp>
// #include <opencv2/highgui.hpp>
// #include<geometry_msgs/Twist.h>
// #include <vector>
// #include <numeric>
// #include <cmath>

// using namespace cv;

// /**
//  * @class PIDController
//  * @brief PID控制器类，用于实现巡线机器人的控制算法
//  */
// class PIDController {
// private:
//     // PID控制参数
//     double kp = 0.2;   // 比例系数
//     double ki = 0.0;    // 积分系数
//     double kd = 0.05;   // 微分系数
    
//     double integral;    // 积分项累计值
//     double error;       // 当前误差值
//     ros::Time prev_time; // 上一次计算时间
//     int stop_flag = 0;   // 停止标志位
//     double prev_error;   // 上一次误差值
//     double straight_line_threshold = 15.0; // 直线判断阈值(像素)
//     int curve_detection_threshold = 100;   // 弯道检测阈值(有效像素数)
//     int curve_state = 0; // 弯道状态：0-直道，1-左弯，2-右弯
    
// public:
//     /**
//      * @brief PID控制器构造函数
//      * @param p 比例系数
//      * @param i 积分系数
//      * @param d 微分系数
//      */
//     PIDController(double p, double i, double d) : 
//         kp(p), ki(i), kd(d), integral(0), error(0) {}
    
//     /**
//      * @brief 重置PID控制器
//      */
//     void reset() {
//         integral = 0;
//     }
    
//     /**
//      * @brief 计算PID输出
//      * @param error 当前误差
//      * @return PID控制输出值
//      */
//     double compute(double error) {
//         ros::Time now = ros::Time::now();
//         double dt = (now - prev_time).toSec(); // 计算时间间隔
//         prev_time = now;
        
//         if (dt <= 0) dt = 0.01; // 最小时间间隔保护
        
//         // 当误差很小时重置积分项，防止积分饱和
//         if (fabs(error) < straight_line_threshold) {
//             integral = 0;
//         } else {
//             integral += error * dt; // 积分项累加
//         }
        
//         double derivative = (error - prev_error) / dt; // 微分项计算
//         prev_error = error;
        
//         // 积分限幅，防止积分项过大
//         integral = std::min(0.5, std::max(-0.5, integral));
        
//         // 当检测到近似直线时，减小P项的影响
//         double effective_kp = kp;
//         if (fabs(error) < straight_line_threshold) {
//             effective_kp *= 0.3;  // 直线时减小比例系数
//         }
        
//         // 弯道状态下增大比例系数
//         if (curve_state == 1 || curve_state == 2) { 
//             effective_kp *= 1.5; // 弯道时更激进的控制
//         }
        
//         return effective_kp * error + ki * integral + kd * derivative;
//     }

//     /**
//      * @brief 巡线核心算法，检测中线并计算误差
//      * @param follow 输出图像(用于可视化)
//      * @param mask 二值化图像
//      * @param error 输出的误差值
//      * @param should_stop 输出的停止标志
//      */
//     void mid(Mat &follow, const Mat &mask, int &error, int &should_stop) {
//         int halfWidth = follow.cols / 2; // 图像中心线位置
//         should_stop = 0;
        
//         // 弯道检测相关变量
//         int left_white_count = 0;  // 左侧白线像素计数(加权)
//         int right_white_count = 0; // 右侧白线像素计数(加权)
        
//         // 使用加权平均误差，近处行权重更大
//         double weighted_error_sum = 0; // 加权误差总和
//         double weight_sum = 0;         // 权重总和
//         int valid_lines = 0;           // 有效检测行数

//         // 从图像底部向上扫描(从近到远)
//         for (int y = follow.rows - 1; y >= 0; --y) {
//             int left = 0, right = follow.cols - 1;
            
//             /******************** 左边界检测 ********************
//              * 从中心向左搜索，找到白线(255)到黑线(0)的过渡点
//              * 要求连续两个像素为黑线以确保检测稳定性
//              */
//             for (int x = halfWidth - 1; x >= 2; --x) {
//                 if (mask.at<uchar>(y, x) == 255 && 
//                     mask.at<uchar>(y, x-1) == 0 && 
//                     mask.at<uchar>(y, x-2) == 0) {
//                     left = x; // 找到左边界
//                     left_white_count += (follow.rows - y); // 加权计数(近处权重更大)
//                     break;
//                 }
//             }
            
//             /******************** 右边界检测 ********************
//              * 从中心向右搜索，找到白线(255)到黑线(0)的过渡点
//              * 同样要求连续两个像素为黑线以确保稳定性
//              */
//             for (int x = halfWidth; x < mask.cols - 2; ++x) {
//                 if (mask.at<uchar>(y, x) == 255 && 
//                     mask.at<uchar>(y, x+1) == 0 && 
//                     mask.at<uchar>(y, x+2) == 0) {
//                     right = x; // 找到右边界
//                     right_white_count += (follow.rows - y); // 加权计数
//                     break;
//                 }
//             }

//             /******************** 中线计算 ********************
//              * 当左右边界都有效时(右>左)，计算中线位置
//              */
//             if (right > left) {  
//                 int mid = (left + right) / 2; // 计算中线位置
//                 follow.at<uchar>(y, mid) = 255; // 在中线位置画白点(可视化)
                
//                 // 计算当前行误差并加权累加(近处行权重更大)
//                 double line_error = halfWidth - mid; // 误差=中心线-实际中线
//                 double weight = (double)(y + 1) / follow.rows;  // 权重计算(越近权重越大)
//                 weighted_error_sum += line_error * weight;
//                 weight_sum += weight;
//                 valid_lines++;
//             }
//         }

//         // 计算加权平均误差(有效行数>0时)
//         error = valid_lines > 0 ? weighted_error_sum / weight_sum : 0;
        
//         // 可视化最后的中线位置(在图像底部画圆标记)
//         if (valid_lines > 0) {
//             int last_mid = halfWidth - error;
//             circle(follow, Point(last_mid, follow.rows - 1), 5, Scalar(255), -1);
//         }
        
//         /******************** 弯道检测逻辑 ********************
//          * 1. 当左侧白线像素不足而右侧充足时，判定为右弯
//          * 2. 当右侧白线像素不足而左侧充足时，判定为左弯
//          * 3. 否则为直道
//          */
//         if (left_white_count < curve_detection_threshold && 
//             right_white_count > curve_detection_threshold) {
//             curve_state = 2; // 右弯状态
//             ROS_WARN("Right curve detected! Left: %d, Right: %d", 
//                     left_white_count, right_white_count);
//         } 
//         else if (right_white_count < curve_detection_threshold && 
//                 left_white_count > curve_detection_threshold) {
//             curve_state = 1; // 左弯状态
//             ROS_WARN("Left curve detected! Left: %d, Right: %d", 
//                     left_white_count, right_white_count);
//         }
//         else {
//             curve_state = 0; // 直道状态
//         }
//     }
// };

// int main(int argc, char **argv) {
//     setlocale(LC_ALL,"");
//     ros::init(argc, argv, "follower_line_pid");
//     ros::NodeHandle nh;
//     ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
//     ros::Rate loop_rate(20); // 控制频率20Hz

//     // 初始化PID控制器
//     PIDController pid_controller(0.2, 0.0, 0.05);
    
//     // 打开摄像头
//     VideoCapture cap("/dev/video0", CAP_V4L2);
//     if (!cap.isOpened()) {
//         ROS_ERROR("Failed to open camera!");
//         return -1;
//     }
//     // 设置摄像头分辨率
//     cap.set(CAP_PROP_FRAME_WIDTH, 640);
//     cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    
//     Mat frame, gray, processed;
//     geometry_msgs::Twist twist; // 速度控制消息
    
//     while(ros::ok()) {
//         cap.read(frame); // 读取摄像头帧
//         if (frame.empty()) continue;
        
//         // 图像处理流程
//         cvtColor(frame, gray, COLOR_BGR2GRAY); // 转换为灰度图
//         threshold(gray, processed, 160, 255, THRESH_BINARY); // 二值化
        
//         int error = 0;
//         int should_stop = 0;
        
//         // 执行巡线算法
//         pid_controller.mid(processed, processed, error, should_stop);
        
//         // 控制逻辑
//         if (should_stop) {
//             // 停止状态
//             twist.linear.x = 0.0;
//             twist.angular.z = 0.0;
//             ROS_WARN("Stopping!");
//         } else {
//             // 动态速度控制 - 误差越大速度越小
//             double base_speed = 0.15; // 基础速度
//             double speed_reduction = 0.08 * (fabs(error)/320.0); // 速度减小量
//             twist.linear.x = std::max(0.1, base_speed - speed_reduction);
            
//             // PID控制计算转向
//             twist.angular.z = pid_controller.compute(error);
            
//             // 速度限幅
//             twist.linear.x = std::min(0.2, std::max(0.1, twist.linear.x));
//             twist.angular.z = std::min(0.5, std::max(-0.5, twist.angular.z)); // 弯道时增大角速度限制
            
//             ROS_DEBUG("Error: %d, Linear: %.2f, Angular: %.2f", 
//                      error, twist.linear.x, twist.angular.z);
//         }
        
//         cmd_pub.publish(twist); // 发布控制指令
//         imshow("Processed", processed); // 显示处理后的图像
//         waitKey(1);
//         loop_rate.sleep();
//     }
    
//     cap.release(); // 释放摄像头
//     return 0;
// }


// #include <ros/ros.h>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>
// #include <std_msgs/Int8.h>
// #include "std_msgs/Int32.h"  // 添加头文件
// #include <std_msgs/Float64.h>
// #include <std_msgs/String.h>
// #include <std_srvs/Empty.h>
// #include <ros/time.h>
// #include <tf/transform_listener.h>
// #include "actionlib/client/simple_goal_state.h"
// #include "actionlib/client/simple_client_goal_state.h"
// #include "actionlib/client/terminal_state.h"
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <sensor_msgs/LaserScan.h>
// #include <sensor_msgs/Image.h>
// #include <opencv2/opencv.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <dynamic_reconfigure/Reconfigure.h>
// #include <dynamic_reconfigure/Config.h>
// #include <sstream>
// #include <iostream>
// #include <stdio.h>
// #include <stdlib.h>
// #include <fstream>
// #include <cstdlib>
// #include <math.h>
// #include "amcl/AMCLConfig.h"
// #include <vector>  
// #include <cmath>  
// #include <limits>  
// #include <map>  
// #include <locale.h>
// #include <boost/shared_ptr.hpp> 
// #include "nav_msgs/Odometry.h"
// #include "tf/transform_listener.h" 
// #include <numeric>

// using namespace cv;
// using namespace std;

// ros::Publisher cmd_pub;
// double Kp = 0.5; // 根据实际情况设置
// double Kd = 0.1; // 根据实际情况设置
// int xun_flag = 1; // 根据实际情况设置
// int stop_flag = 0;

// void mid(Mat &follow, const Mat &mask, int &error) {
//     int halfWidth = follow.cols / 2;
//     int half = halfWidth;
//     int mid_output = half;

//     for (int y = follow.rows - 1; y >= 0; --y) {
//         int left = 0, right = follow.cols;
        
//         Mat leftPart = mask.row(y).colRange(max(0, half - halfWidth), half);
//         if (countNonZero(leftPart) == 0) {
//             left = max(0, half - halfWidth);
//         } else {
//             vector<int> leftIndices;
//             for (int x = 0; x < half; ++x) {
//                 if (mask.at<uchar>(y, x) == 255) {
//                     leftIndices.push_back(x);
//                 }
//             }
//             if (!leftIndices.empty()) {
//                 left = std::accumulate(leftIndices.begin(), leftIndices.end(), 0.0) / leftIndices.size();
//             }
//         }

//         Mat rightPart = mask.row(y).colRange(half, min(follow.cols, half + halfWidth));
//         if (countNonZero(rightPart) == 0) {
//             right = min(follow.cols, half + halfWidth);
//         } else {
//             vector<int> rightIndices;
//             for (int x = half; x < follow.cols; ++x) {
//                 if (mask.at<uchar>(y, x) == 255) {
//                     rightIndices.push_back(x);
//                 }
//             }
//             if (!rightIndices.empty()) {
//                 right = std::accumulate(rightIndices.begin(), rightIndices.end(), 0.0) / rightIndices.size();
//             }
//         }

//         if (y == 245) {   //在235行如果白色像素点多余200（640中的200），触发停车标志位
//             int WhiteCount = 0;
//             for (int x = 0; x < mask.cols; ++x) {
//                 if (mask.at<uchar>(y, x) == 255) {  // 计算白色像素点数量
//                     WhiteCount++;
//                 }
//             }   
//             if (WhiteCount >= 180 ) {
//                 stop_flag = 1;
//                 cout<<"stop_flag = "<<stop_flag<<endl;
//             }
//         }
        
//         int mid = (left + right) / 2;
//         half = mid;
//         follow.at<uchar>(y, mid) = 255;

//         if (y == 235) {
//             mid_output = mid;
//         }

//     }

//     circle(follow, Point(mid_output, 235), 5, Scalar(255), -1);
//     error = follow.cols / 2 - mid_output;
// }

// void imageCallback(const Mat &msg) {
//     static double previous_error = 0.0;

//     // cv_bridge::CvImagePtr cv_ptr;
//     // try {
//     //     // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     //     cv_ptr = msg;
//     // } catch (cv_bridge::Exception &e) {
//     //     ROS_ERROR("cv_bridge exception: %s", e.what());
//     //     return;
//     // }

//     // Mat img = cv_ptr->image;
//     Mat img = msg;
//     int y_start = 170;
//     int y_end = y_start + 310;
//     Mat cropped_img = img(Range(y_start, y_end), Range::all());

//     Mat img_hsv;
//     cvtColor(cropped_img, img_hsv, COLOR_BGR2HSV);
//     Mat mask;
//     inRange(img_hsv, Scalar(0, 0, 210), Scalar(179, 30, 255), mask);

//     Mat follow = mask.clone();
//     int error;
//     mid(follow, mask, error);

//     double derivative = error - previous_error;
//     double control_signal = Kp * error + Kd * derivative;

//     if (abs(error) <= 10) {
//         error = 0;
//     }

//     previous_error = error;
//     // cout<<error<<endl;
//     geometry_msgs::Twist twist;
//     if(stop_flag){    // 停车标志位
//         stop_flag ++ ;
//         if(stop_flag >= 10)
//         {
//             twist.linear.x = 0;
//             twist.angular.z = 0;
//         }else{
//             twist.linear.x = 0.4;
//             twist.angular.z = -control_signal;
//         }
//         cout<<stop_flag<<endl;
//     }else{
// 	    twist.linear.x = 0.4;
//     	twist.angular.z = -control_signal;
//     }
//     ROS_INFO("Publishing Twist: linear.x=%f, angular.z=%f", twist.linear.x, twist.angular.z);
//     cmd_pub.publish(twist);

//     // imshow("img", img);
//     // imshow("mask", mask);
//     // imshow("follow", follow);
//     // waitKey(1);
// }

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "image_processor");
//     ros::NodeHandle nh;
//     ros::Rate loop_rate(30); // 10 Hz 
//     cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

//     while (ros::ok()) {
//         cv::VideoCapture cap("/dev/video0");
//         if (!cap.isOpened()) {
//         std::cerr << "Error: Could not open video capture device." << std::endl;
//         return -1;
//         }
//         cv::Mat frame;
//         bool rec = cap.read(frame);
//         cap >> frame;
//         if (rec) {
//             ROS_ERROR("Captured empty frame");
//             continue;
//         }
        
//         imageCallback(frame);
//         ROS_INFO("111");
//         loop_rate.sleep();
//         // ros::spinOnce();
//     }
//     return 0;
// }

