#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "qr_detector_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    
    // 创建图像发布者（用于可视化调试）
    image_transport::Publisher pub = it.advertise("qr_detector/debug_image", 1);
    if (!pub) {
        ROS_ERROR("无法初始化话题发布者！");
        return -1;
    }
    ROS_INFO("话题发布者初始化成功");
    
    // 初始化OpenCV摄像头
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        ROS_ERROR("无法打开摄像头！");
        return -1;
    }
    
    // 配置摄像头参数
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    
    // 初始化ZBar扫描器
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);  // 启用所有符号
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1); // 特别启用QR码

    cv::Mat frame, gray;
    while (ros::ok()) {
        cap >> frame;
        if (frame.empty()) {
            ROS_WARN_THROTTLE(5, "接收到空帧");
            continue;
        }
        
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
                ROS_INFO_STREAM("检测到二维码: " << symbol->get_data());
                
                // 在图像上绘制二维码位置
                std::vector<cv::Point> points;
                for(int i=0; i<symbol->get_location_size(); i++) {
                    points.push_back(cv::Point(
                        symbol->get_location_x(i),
                        symbol->get_location_y(i)
                    ));
                }
                cv::polylines(frame, points, true, cv::Scalar(0,255,0), 2);
            }
        }
        
        // 发布带标记的图像（用于调试）
        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
        //     std_msgs::Header(), "bgr8", frame
        // ).toImageMsg();
        // pub.publish(msg);
        
        ros::spinOnce();
    }
    
    cap.release();
    return 0;
}