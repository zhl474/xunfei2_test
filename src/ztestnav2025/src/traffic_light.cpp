#include "ztestnav2025/traffic_light.h"

using namespace cv;
using namespace std;

// 定义颜色范围（HSV）
Scalar lower_red = Scalar(0, 100, 100); // 红色下限
Scalar upper_red = Scalar(10, 255, 255); // 红色上限
Scalar lower_green = Scalar(50, 100, 100); // 绿色下限
Scalar upper_green = Scalar(80, 255, 255); // 绿色上限

// 判断红绿灯状态
int detectTrafficLightStatus() {
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        ROS_ERROR("无法打开摄像头！");
        return -1;
    }
    
    // 配置摄像头参数
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // 转换为 HSV 颜色空间
    Mat hsv_frame,frame;
    cap >> frame;
    if (frame.empty()) {
        ROS_WARN_THROTTLE(5, "接收到空帧");
        return false;
    }
    cv::flip(frame,frame, 1);
    cvtColor(frame, hsv_frame, COLOR_BGR2HSV);

    // 创建红色和绿色的掩膜
    Mat red_mask, green_mask;
    inRange(hsv_frame, lower_red, upper_red, red_mask);
    inRange(hsv_frame, lower_green, upper_green, green_mask);

    // 形态学操作：闭运算，去除噪声
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(red_mask, red_mask, MORPH_CLOSE, kernel);
    morphologyEx(green_mask, green_mask, MORPH_CLOSE, kernel);

    // 寻找轮廓
    vector<vector<Point>> red_contours, green_contours;
    findContours(red_mask, red_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    findContours(green_mask, green_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    int status = 0;

    // 检查红色区域
    for (const auto& contour : red_contours) {
        double area = contourArea(contour);
        if (area < 100) continue;

        Point2f center;
        float radius;
        minEnclosingCircle(contour, center, radius);
        double circle_area = CV_PI * radius * radius;
        double roundness = area / circle_area;  // 计算圆度 [[3]]

        // 圆度判断
        if (roundness > 0.5 && roundness < 1.0 && radius > 10) {
            Rect rbox = boundingRect(contour);
            rectangle(frame, rbox, Scalar(0, 0, 255), 2);  // 绘制红色边界框 [[3]]

            // 格式化文本并绘制到图像上 [[6]]
            string info = "Red: R=" + to_string((int)radius) +
                ", A=" + to_string((int)area) +
                ", Round=" + to_string(roundness).substr(0, 4);
            putText(frame, info, Point(center.x - 50, center.y - 20),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
            status = 1;
        }
    }

    // 检查绿色区域
    for (const auto& contour : green_contours) {
        double area = contourArea(contour);
        if (area < 100) continue;

        Point2f center;
        float radius;
        minEnclosingCircle(contour, center, radius);
        double circle_area = CV_PI * radius * radius;
        double roundness = area / circle_area;  // 计算圆度 [[3]]

        if (roundness > 0.5 && roundness < 1.0 && radius > 10) {
            Rect gbox = boundingRect(contour);
            rectangle(frame, gbox, Scalar(0, 255, 0), 2);  // 绘制绿色边界框 [[4]]

            // 格式化文本并绘制到图像上 [[6]]
            string info = "Green: R=" + to_string((int)radius) +
                ", A=" + to_string((int)area) +
                ", Round=" + to_string(roundness).substr(0, 4);
            putText(frame, info, Point(center.x - 50, center.y - 20),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
            status = 2;
        }
    }

    imshow("Detection Result", frame);
    waitKey(1);
    cap.release();

    return status;
}

int main() {
    // 读取图像
    Mat image = imread("C:\\Users\\Cyrus\\Desktop\\1.jpg");
    if (image.empty()) {
        cout << "Error: Could not open or find the image!" << endl;
        return -1;
    }

    // 调用检测函数
    int status = detectTrafficLightStatus();

    // 显示结果
    cout << "Traffic Light Status: " << status << endl;

    // 可视化结果（可选）
    imshow("Detection Result", image);
    waitKey(0);

    return 0;
}