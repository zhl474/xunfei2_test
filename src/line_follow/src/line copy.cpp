#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>

using namespace cv;

// 全局变量
double twist_linear_x = 0.4;
double twist_angular_z = 0.0;
sensor_msgs::ImagePtr processed_image_msg;
int stop_flag = 0;

class PIDController {
private:
    double Kp, Kd;
    double prev_error;
    
public:
    PIDController(double p, double d) : Kp(p), Kd(d), prev_error(0) {}
    
    void reset() {
        prev_error = 0;
    }
    
    double compute(double error) {
        double derivative = error - prev_error;
        prev_error = error;
        return Kp * error + Kd * derivative;
    }
};

void mid(Mat &follow, const Mat &mask, int &error) {
    int halfWidth = follow.cols / 2;
    int half = halfWidth;
    int mid_output = half;

    for (int y = follow.rows - 1; y >= 0; --y) {
        int left = 0, right = follow.cols;
        
        // 左半部分处理
        Mat leftPart = mask.row(y).colRange(std::max(0, half - halfWidth), half);
        if (countNonZero(leftPart) == 0) {
            left = std::max(0, half - halfWidth);
        } else {
            std::vector<int> leftIndices;
            for (int x = 0; x < half; ++x) {
                if (mask.at<uchar>(y, x) == 255) {
                    leftIndices.push_back(x);
                }
            }
            if (!leftIndices.empty()) {
                left = std::accumulate(leftIndices.begin(), leftIndices.end(), 0.0) / leftIndices.size();
            }
        }ROS_INFO("左边边界%d",left);

        // 右半部分处理
        Mat rightPart = mask.row(y).colRange(half, std::min(follow.cols, half + halfWidth));
        if (countNonZero(rightPart) == 0) {
            right = std::min(follow.cols, half + halfWidth);
        } else {
            std::vector<int> rightIndices;
            for (int x = half; x < follow.cols; ++x) {
                if (mask.at<uchar>(y, x) == 255) {
                    rightIndices.push_back(x);
                }
            }
            if (!rightIndices.empty()) {
                right = std::accumulate(rightIndices.begin(), rightIndices.end(), 0.0) / rightIndices.size();
            }
        }ROS_INFO("右边边界%d",right);

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
    

    circle(follow, Point(mid_output, 235), 5, Scalar(255), -1);
    error = follow.cols / 2 - mid_output;
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg, PIDController* pid_controller) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        Mat img = cv_ptr->image;
        
        // ROI裁剪
        int y_start = 170;
        int y_end = y_start + 310;
        Mat cropped_img = img(Range(y_start, y_end), Range::all());

        // HSV颜色空间转换
        Mat img_hsv, mask;
        cvtColor(cropped_img, img_hsv, COLOR_BGR2HSV);
        inRange(img_hsv, Scalar(0, 0, 210), Scalar(179, 30, 255), mask);

        // 中线检测
        Mat follow = mask.clone();
        int error;
        mid(follow, mask, error);

        // PID控制
        double control_signal = pid_controller->compute(error);
        if (abs(error) <= 10) error = 0;

        // 运动控制逻辑
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

        // 发布处理后的图像
        processed_image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", follow).toImageMsg();
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "line_follower");
    ros::NodeHandle nh;
    
    PIDController pid_controller(0.5, 0.1);

    ros::Subscriber img_sub = nh.subscribe<sensor_msgs::Image>(
        "/usb_cam/image_raw", 10, boost::bind(imageCallback, _1, &pid_controller));
    
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/processed_image", 10);

    ros::Rate loop_rate(30);
    
    while(ros::ok()) {
        geometry_msgs::Twist twist;
        twist.linear.x = twist_linear_x;
        twist.angular.z = twist_angular_z;
        
        cmd_pub.publish(twist);
        if (processed_image_msg) {
            img_pub.publish(processed_image_msg);
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}