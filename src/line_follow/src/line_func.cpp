#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include "line_follow/line_follow.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

using namespace cv;
using namespace std;

// 删除原otsu()函数，改用HSV阈值处理
void processImage(const Mat& src, Mat& mask, Mat& follow) {
    // HSV颜色阈值处理
    Mat img_hsv;
    cvtColor(src, img_hsv, COLOR_BGR2HSV);
    inRange(img_hsv, Scalar(0, 0, 200), Scalar(179, 30, 255), mask); // 保持原有阈值
    
    follow = mask.clone(); // 保持原有复制操作
}

class PIDController {
private:
    double kp = 0.2, ki = 0.0, kd = 0.05;  // 更温和的PID参数
    double integral;
    double error;
    ros::Time prev_time;
    int stop_flag = 0;
    double prev_error;
    double straight_line_threshold = 15.0; // 直线判断阈值(像素)

    ros::NodeHandle nh_;
    ros::ServiceServer line_server_;
        
    ros::Publisher cmd_pub;
    ros::Rate loop_rate;

    Mat frame, gray, processed;
    ros::Subscriber lineimage_sub_;
    sensor_msgs::ImageConstPtr latest_img_;  // 智能指针成员变量

    geometry_msgs::Twist twist;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        latest_img_ = msg;
    }
    
    
public:
    PIDController(double p, double i, double d) : 
        kp(p), ki(i), kd(d), integral(0), error(0),
        nh_(),loop_rate(20)
        {
            lineimage_sub_ = nh_.subscribe("/usb_cam/image_raw", 1, &PIDController::imageCallback, this);
            cmd_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 5);
            line_server_ = nh_.advertiseService("line_server", &PIDController::line_follow_start, this);
            ROS_INFO("视觉巡线初始化");
        }
    
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

        // 修改循环条件，防止数组越界
        for (int y = follow.rows - 1; y >= 200; --y) {
            int left = 0, right = follow.cols - 1;
            
            // 左边界检测
            for (int x = halfWidth - 1; x >= 2; --x) {
                if (mask.at<uchar>(y, x) == 255 && 
                    mask.at<uchar>(y, x-1) == 0 && 
                    mask.at<uchar>(y, x-2) == 0) {
                    left = x;
                    break;
                }
            }
            
            // 右边界检测
            for (int x = halfWidth; x < mask.cols - 2; ++x) {
                if (mask.at<uchar>(y, x) == 255 && 
                    mask.at<uchar>(y, x+1) == 0 && 
                    mask.at<uchar>(y, x+2) == 0) {
                    right = x;
                    break;
                }
            }
            
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
        
        // 可视化最后的中线位置
        if (valid_lines > 0) {
            int last_mid = halfWidth - error;
            circle(follow, Point(last_mid, follow.rows - 1), 5, Scalar(255), -1);
        }

    }

    bool line_follow_start(line_follow::line_follow::Request& req,line_follow::line_follow::Response& resp){
        while(ros::ok()) {
            if (!latest_img_) {
                ROS_ERROR("尚未收到图像数据！");
                resp.line_follow_done = 0;
                return true;  // 仍需返回true表示服务调用完成
            }
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(latest_img_, sensor_msgs::image_encodings::BGR8);
            const cv::Mat& frame = cv_ptr->image;
            
            // 保持原有预处理
            Rect roi(0, 200, frame.cols, frame.rows - 200);
            Mat cropped_frame = frame(roi);
            flip(cropped_frame, cropped_frame, 1);

            // ========== 替换二值化处理 ==========
            Mat mask, processed;
            processImage(cropped_frame, mask, processed); // 输出mask和processed
            
            // 保持原有后续处理
            int error = 0;
            int should_stop = 0;
            mid(processed, mask, error, should_stop); // 注意参数顺序
            
            // [保持原有的控制逻辑不变]
            if (should_stop) {
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                ROS_WARN("Stopping!");
                break;
            } else {
                double base_speed = 0.15;
                double speed_reduction = 0.08 * (fabs(error)/320.0);
                twist.linear.x = std::max(0.1, base_speed - speed_reduction);
                
                twist.angular.z = compute(error);
                
                // 限幅
                twist.linear.x = std::min(0.2, std::max(0.1, twist.linear.x));
                twist.angular.z = std::min(0.3, std::max(-0.3, twist.angular.z));
                
                ROS_DEBUG("Error: %d, Linear: %.2f, Angular: %.2f", 
                        error, twist.linear.x, twist.angular.z);
            }

            ROS_INFO("x速度%f",twist.linear.x);
            ROS_INFO("z速度%f",twist.angular.z);
            
            cmd_pub.publish(twist);
            imshow("Processed", processed);
            waitKey(1);
            loop_rate.sleep();
        }
        return true;
    }
};


int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "follower_line");

    PIDController pid_controller(0.2, 0.0, 0.05);  // 使用更温和的参数
    ros::spin();
    return 0;
}