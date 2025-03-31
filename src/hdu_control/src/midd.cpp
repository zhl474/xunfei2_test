#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <numeric>

using namespace cv;
using namespace std;

// PID控制参数
const double Kp = 0.00695; //0.00595
const double Kd = 0.0035; //0.0009
double previous_error = 0.0;
int stop_flag = 0;
ros::Publisher cmd_pub;
// 图像分辨率 640*480

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

void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    static double previous_error = 0.0;

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat img = cv_ptr->image;
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

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "lane_follower");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 1, imageCallback);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    ros::Rate loop_rate(30); // 30 Hz

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

