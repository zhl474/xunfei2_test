#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int8.h>
#include "std_msgs/Int32.h"  // 添加头文件
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include "actionlib/client/simple_goal_state.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "actionlib/client/terminal_state.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/OccupancyGrid.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <cstdlib>
#include <math.h>
#include "amcl/AMCLConfig.h"
#include <vector>  
#include <cmath>  
#include <limits>  
#include <map>  
#include <locale.h>
#include <boost/shared_ptr.hpp> 
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h" 
#include <numeric>

using namespace cv;
using namespace std;

ros::Publisher cmd_pub;
double Kp = 0.5; // 根据实际情况设置
double Kd = 0.1; // 根据实际情况设置
int xun_flag = 1; // 根据实际情况设置
int stop_flag = 0;

void mid(Mat &follow, const Mat &mask, int &error) {
    int halfWidth = follow.cols / 2;
    int half = halfWidth;
    int mid_output = half;

    for (int y = follow.rows - 1; y >= 0; --y) {
        int left = 0, right = follow.cols;
        
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
                left = std::accumulate(leftIndices.begin(), leftIndices.end(), 0.0) / leftIndices.size();
            }
        }

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
                right = std::accumulate(rightIndices.begin(), rightIndices.end(), 0.0) / rightIndices.size();
            }
        }

        if (y == 245) {   //在235行如果白色像素点多余200（640中的200），触发停车标志位
            int WhiteCount = 0;
            for (int x = 0; x < mask.cols; ++x) {
                if (mask.at<uchar>(y, x) == 255) {  // 计算白色像素点数量
                    WhiteCount++;
                }
            }   
            if (WhiteCount >= 180 ) {
                stop_flag = 1;
                cout<<"stop_flag = "<<stop_flag<<endl;
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

void imageCallback(const Mat &msg) {
    static double previous_error = 0.0;

    // cv_bridge::CvImagePtr cv_ptr;
    // try {
    //     // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //     cv_ptr = msg;
    // } catch (cv_bridge::Exception &e) {
    //     ROS_ERROR("cv_bridge exception: %s", e.what());
    //     return;
    // }

    // Mat img = cv_ptr->image;
    Mat img = msg;
    int y_start = 170;
    int y_end = y_start + 310;
    Mat cropped_img = img(Range(y_start, y_end), Range::all());

    Mat img_hsv;
    cvtColor(cropped_img, img_hsv, COLOR_BGR2HSV);
    Mat mask;
    inRange(img_hsv, Scalar(0, 0, 210), Scalar(179, 30, 255), mask);

    Mat follow = mask.clone();
    int error;
    mid(follow, mask, error);

    double derivative = error - previous_error;
    double control_signal = Kp * error + Kd * derivative;

    if (abs(error) <= 10) {
        error = 0;
    }

    previous_error = error;
    // cout<<error<<endl;
    geometry_msgs::Twist twist;
    if(stop_flag){    // 停车标志位
        stop_flag ++ ;
        if(stop_flag >= 10)
        {
            twist.linear.x = 0;
            twist.angular.z = 0;
        }else{
            twist.linear.x = 0.4;
            twist.angular.z = -control_signal;
        }
        cout<<stop_flag<<endl;
    }else{
	    twist.linear.x = 0.4;
    	twist.angular.z = -control_signal;
    }
    ROS_INFO("Publishing Twist: linear.x=%f, angular.z=%f", twist.linear.x, twist.angular.z);
    cmd_pub.publish(twist);

    // imshow("img", img);
    // imshow("mask", mask);
    // imshow("follow", follow);
    // waitKey(1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30); // 10 Hz 
    cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    while (ros::ok()) {
        cv::VideoCapture cap("/dev/video0");
        if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video capture device." << std::endl;
        return -1;
        }
        cv::Mat frame;
        bool rec = cap.read(frame);
        cap >> frame;
        if (rec) {
            ROS_ERROR("Captured empty frame");
            continue;
        }
        
        imageCallback(frame);
        ROS_INFO("111");
        loop_rate.sleep();
        // ros::spinOnce();
    }
    return 0;
}
