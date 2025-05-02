// ROS 核心头文件
#include <ros/ros.h>
// ROS 图像消息类型
#include <sensor_msgs/Image.h>
// ROS 速度控制消息类型
#include <geometry_msgs/Twist.h>
// ROS 与 OpenCV 图像转换接口
#include <cv_bridge/cv_bridge.h>
// OpenCV 核心功能
#include <opencv2/opencv.hpp>
// 标准库头文件
#include <vector>
#include <numeric>

// 使用命名空间
using namespace cv;
using namespace std;

// ========== 全局变量 ==========
double twist_linear_x = 0.4;       // 存储计算出的线速度
double twist_angular_z = 0.0;      // 存储计算出的角速度
sensor_msgs::Image processed_image;  // 存储处理后的图像（用于发布）
int stop_flag = 0;                 // 停车标志位

// ========== PID 控制器类 ==========
class PIDController {
private:
    double Kp, Kd;  // 只需要比例和微分
    double prev_error;   // 上一次误差值
    
public:
    // 构造函数初始化 PID 参数
    PIDController(double p, double d) : Kp(p), Kd(d), prev_error(0) {}
    
    // 重置 PID 控制器状态
    void reset() {
        prev_error = 0;
    }
    
    // 计算 PD 输出
    double compute(double error) {
        double derivative = error - prev_error;
        prev_error = error;
        
        // 返回 PD 计算结果
        return Kp * error + Kd * derivative;
    }
};

// ========== 中线检测函数 ==========
void mid(Mat &follow, const Mat &mask, int &error) {
    int halfWidth = follow.cols / 2;
    int half = halfWidth;
    int mid_output = half;

    for (int y = follow.rows - 1; y >= 0; --y) {
        int left = 0, right = follow.cols;
        
        // 处理左半部分
        Mat leftPart = mask.row(y).colRange(max(0, half - halfWidth), half);
        if (countNonZero(leftPart) == 0) {
            left = max(0, half - halfWidth);
        } else {
            vector<int> leftIndices;
            for (int x = 0; x < half; ++x) {
                if (mask.at<uchar>(y, x) == 255) {
                    leftIndices.push_back(x);
                }
            }
            if (!leftIndices.empty()) {
                left = accumulate(leftIndices.begin(), leftIndices.end(), 0.0) / leftIndices.size();
            }
        }

        // 处理右半部分
        Mat rightPart = mask.row(y).colRange(half, min(follow.cols, half + halfWidth));
        if (countNonZero(rightPart) == 0) {
            right = min(follow.cols, half + halfWidth);
        } else {
            vector<int> rightIndices;
            for (int x = half; x < follow.cols; ++x) {
                if (mask.at<uchar>(y, x) == 255) {
                    rightIndices.push_back(x);
                }
            }
            if (!rightIndices.empty()) {
                right = accumulate(rightIndices.begin(), rightIndices.end(), 0.0) / rightIndices.size();
            }
        }

        // 在特定行检查是否需要停车
        if (y == 245) {
            int WhiteCount = 0;
            for (int x = 0; x < mask.cols; ++x) {
                if (mask.at<uchar>(y, x) == 255) {
                    WhiteCount++;
                }
            }   
            if (WhiteCount >= 180) {
                stop_flag = 1;
                ROS_INFO("Stop flag set: %d", stop_flag);
            }
        }
        
        int mid = (left + right) / 2;
        half = mid;
        follow.at<uchar>(y, mid) = 255;

        if (y == 235) {
            mid_output = mid;
        }
    }

    circle(follow, Point(mid_output, 235), 5, Scalar(255), -1);
    error = follow.cols / 2 - mid_output;
}

// ========== 图像回调函数 ==========
void image_Callback(const sensor_msgs::ImageConstPtr& msg, PIDController* pid_controller) {
    // 将 ROS 图像消息转换为 OpenCV 格式
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("图像转换失败: %s", e.what());
        return;
    }
    
    // 获取原始图像并裁剪
    Mat img = cv_ptr->image;
    int y_start = 170;
    int y_end = y_start + 310;
    Mat cropped_img = img(Range(y_start, y_end), Range::all());

    // 转换为HSV颜色空间并创建掩膜
    Mat img_hsv, mask;
    cvtColor(cropped_img, img_hsv, COLOR_BGR2HSV);
    inRange(img_hsv, Scalar(0, 0, 210), Scalar(179, 30, 255), mask);

    // 中线检测
    Mat follow = mask.clone();
    int error;
    mid(follow, mask, error);

    // 使用PD控制器计算控制信号
    double control_signal = pid_controller->compute(error);

    // 如果偏差很小则忽略
    if (abs(error) <= 10) {
        error = 0;
    }

    // 处理停车逻辑
    if(stop_flag) {
        stop_flag++;
        if(stop_flag >= 10) {
            twist_linear_x = 0;
            twist_angular_z = 0;
        } else {
            twist_linear_x = 0.4;
            twist_angular_z = -control_signal;
        }
        ROS_INFO("Stopping... flag: %d", stop_flag);
    } else {
        twist_linear_x = 0.4;
        twist_angular_z = -control_signal;
    }

    // 将处理后的图像转换为ROS消息
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(
        std_msgs::Header(), "mono8", follow).toImageMsg();
    processed_image = *img_msg;
}

// ========== 主函数 ==========
int main(int argc, char **argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "follower_line_pid");
    ros::NodeHandle nh;
    
    // 初始化 PID 控制器
    PIDController pid_controller(0.5, 0.1);  // Kp=0.5, Kd=0.1

    // 创建订阅者（使用boost::bind传递PID控制器指针）
    ros::Subscriber img_sub = nh.subscribe<sensor_msgs::Image>(
        "/usb_cam/image_raw", 
        10, 
        boost::bind(image_Callback, _1, &pid_controller)
    );
    
    // 创建发布者
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/processed_image", 10);

    // 设置循环频率 (30Hz)
    ros::Rate loop_rate(30);
    
    // 主循环
    while(ros::ok()) {
        // 创建并发布速度指令
        geometry_msgs::Twist twist;
        twist.linear.x = twist_linear_x;
        twist.angular.z = twist_angular_z;
        cmd_pub.publish(twist);
        
        // 发布处理后的图像
        img_pub.publish(processed_image);
        
        // ROS 回调处理
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}