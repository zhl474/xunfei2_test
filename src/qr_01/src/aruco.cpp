#include <ros/ros.h>
#include "qr_01/qr_srv.h"
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <string>

bool qr_detect(qr_01::qr_srv::Request& req,qr_01::qr_srv::Response& resp){
    // 初始化OpenCV摄像头 - 修改为使用v4l2后端
    cv::VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        ROS_ERROR("无法打开摄像头！");
        return -1;
    }
    
    // 配置摄像头参数
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    
    // 检查摄像头参数设置是否成功
    double width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    ROS_INFO_STREAM("摄像头实际分辨率: " << width << "x" << height);
    
    // 初始化ZBar扫描器
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);  // 启用所有符号
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1); // 特别启用QR码

    cv::Mat frame, gray;
    cap >> frame;
    if (frame.empty()) {
        ROS_WARN_THROTTLE(5, "接收到空帧");
        return false;
    }
    cv::flip(frame,frame, 1);
    cv::imshow("QR",frame);
    cv::waitKey(0);
    // 转换为灰度图（ZBar需要Y800格式）
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // 包装图像数据供ZBar使用
    zbar::Image zbar_image(gray.cols, gray.rows, "Y800", 
                            gray.data, gray.cols * gray.rows);
    
    // 扫描二维码
    int detected = scanner.scan(zbar_image);
    
    // 处理检测结果
    if (detected > 0) {
        for(zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
            symbol != zbar_image.symbol_end(); ++symbol) {
            // 输出二维码内容
            std::string qr_content  = symbol->get_data();
            ROS_INFO_STREAM("检测到二维码: " << qr_content);
            if (qr_content == "Vegetable"){
                resp.qr_result = 1;
            }
            else if (qr_content == "Fruit"){
                resp.qr_result = 2;
            }
            else if (qr_content == "Dessert"){
                resp.qr_result = 3;
            }
            else{
                resp.qr_result = 0;
            }
        }
    }
    cap.release();
    return true;
}

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "qr_detector_node");
    ros::NodeHandle nh;
    ros::ServiceServer qr_server = nh.advertiseService("qr_detect",qr_detect);
    ROS_INFO("qr服务已经启动");
    ros::spin();
}