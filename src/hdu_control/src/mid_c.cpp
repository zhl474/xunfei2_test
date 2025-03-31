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
const double Kp = 0.00175;
const double Kd = 0.0006;
double previous_error = 0.0;

ros::Publisher cmd_pub;

void mid(Mat &follow, const Mat &mask, int &error) {
    int halfWidth = follow.cols / 2;
    int half = halfWidth;
    int mid_output = half;

    for (int y = follow.rows - 1; y >= 0; --y) {
        int left, right;
        
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

        int mid = (left + right) / 2;
        half = mid;
        follow.at<uchar>(y, mid) = 255;

        if (y == 170) {
            mid_output = mid;
        }
    }

    circle(follow, Point(mid_output, 170), 5, Scalar(255), -1);
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
    int y_start = (img.rows - 300) / 2;
    int y_end = y_start + 270;
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

    if (abs(error) <= 5) {
        error = 0;
    }

    previous_error = error;

    geometry_msgs::Twist twist;
    twist.linear.x = 0.1;
    twist.angular.z = -control_signal;
    cmd_pub.publish(twist);

    // imshow("img", img);
    // imshow("mask", mask);
    // imshow("follow", follow);
    waitKey(1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lane_follower");
    ros::NodeHandle nh;

    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Subscriber sub = nh.subscribe("/usb_cam/image_raw", 1, imageCallback);

    ros::spin();
    return 0;
}

