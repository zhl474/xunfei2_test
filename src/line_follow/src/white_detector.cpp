#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include <geometry_msgs/Twist.h>
#include "line_follow/line_follow.h"
#include "ztestnav2025/getpose_server.h"

using namespace cv;
using namespace std;

// 自适应HSV阈值处理
void processImage(const Mat& src, Mat& mask, Mat& follow) {
    // 转换到HSV空间
    Mat img_hsv;
    cvtColor(src, img_hsv, COLOR_BGR2HSV);
    
    // 分割HSV通道
    vector<Mat> hsv_channels;
    split(img_hsv, hsv_channels);
    
    // 计算V通道的直方图
    Mat hist;
    const int histSize = 256;
    float range[] = { 0, 256 };
    const float* histRange = { range };
    calcHist(&hsv_channels[2], 1, 0, Mat(), hist, 1, &histSize, &histRange);
    
    // 找到V通道的主要亮度范围（避开极低值）
    int v_min = 200;  // 默认最小值
    for (int i = 100; i < 256; i++) {  // 从100开始避免纯黑色
        if (hist.at<float>(i) > src.total() * 0.01) {  // 至少1%的像素
            v_min = i;
            break;
        }
    }
    v_min = max(180, v_min);  // 确保最小值不低于180
    
    // 动态调整S通道阈值（白色区域通常饱和度较低）
    int s_max = 30;  // 默认最大值
    double s_mean = mean(hsv_channels[1])[0];
    s_max = min(50, static_cast<int>(s_mean * 1.5));  // 不超过均值的1.5倍
    
    // 应用动态阈值
    inRange(img_hsv, Scalar(0, 0, v_min), Scalar(179, s_max, 255), mask);
    
    // 形态学操作优化掩码
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(mask, mask, MORPH_OPEN, kernel);  // 开运算去除小噪点
    morphologyEx(mask, mask, MORPH_CLOSE, kernel); // 闭运算填充小孔
    
    // 可视化调试信息
    putText(mask, format("V min: %d", v_min), Point(10, 30), 
            FONT_HERSHEY_SIMPLEX, 0.7, Scalar(128), 2);
    putText(mask, format("S max: %d", s_max), Point(10, 60), 
            FONT_HERSHEY_SIMPLEX, 0.7, Scalar(128), 2);
    
    follow = mask.clone();
}

class PIDController {
private:
    double kp = 0.2, ki = 0.0, kd = 0.05;  // PID参数
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
    geometry_msgs::Twist twist;
    
    ros::ServiceClient poseget_client;
    ztestnav2025::getpose_server pose_result;
    
    // 自适应阈值参数
    int adaptive_v_min = 200;
    int adaptive_s_max = 30;
    int frame_count = 0;
    
public:
    PIDController(double p, double i, double d) : 
        kp(p), ki(i), kd(d), integral(0), error(0),
        nh_(),loop_rate(20)
        {
            ROS_INFO("等待坐标获取服务中---");
            poseget_client = nh_.serviceClient<ztestnav2025::getpose_server>("getpose_server");
            poseget_client.waitForExistence();
            
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

        // 如果连续多行没有检测到白色区域，触发停止
        if (valid_lines < 3) {
            should_stop = 1;
        }
    }

    bool line_follow_start(line_follow::line_follow::Request& req,line_follow::line_follow::Response& resp){
        VideoCapture cap;
        cap.open("/dev/video0", cv::CAP_V4L2);
        if (!cap.isOpened()) {
            ROS_ERROR("无法打开摄像头");
            throw std::runtime_error("Camera initialization failed");
        }
        cap.set(CAP_PROP_FRAME_WIDTH, 640);
        cap.set(CAP_PROP_FRAME_HEIGHT, 480);
        
        // 初始化时间
        prev_time = ros::Time::now();
        
        while(ros::ok()) {
            if (poseget_client.call(pose_result)){
                ROS_INFO("小车坐标xyz:%f,%f,%f",pose_result.response.pose_at[0],pose_result.response.pose_at[1],pose_result.response.pose_at[2]);
                if(pose_result.response.pose_at[1]<0.3){
                    ROS_WARN("定位停止");
                    break;
                }
            }
            else{
                ROS_ERROR("获取位姿失败");
            }
            
            cap.read(frame);
            if (frame.empty()) continue;
            
            // 保持原有预处理
            Rect roi(0, 200, frame.cols, frame.rows - 200);
            Mat cropped_frame = frame(roi);
            flip(cropped_frame, cropped_frame, 1);

            // ========== 二值化处理 ==========
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

            // ROS_INFO("x速度%f",twist.linear.x);
            // ROS_INFO("z速度%f",twist.angular.z);
            
            cmd_pub.publish(twist);
            imshow("Processed", processed);
            waitKey(1);
            loop_rate.sleep();
            
            frame_count++;
        }
        cap.release();
        return true;
    }
};


int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "white_detector");

    PIDController pid_controller(0.2, 0.0, 0.05);  // 使用更温和的参数
    ros::spin();
    return 0;
}
