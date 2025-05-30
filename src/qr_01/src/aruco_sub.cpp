// #include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/highgui/highgui.hpp>

// // 图像回调函数
// void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
//     try {
//         // 将ROS图像消息转换为OpenCV格式（BGR8）
//         cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        
//         // 显示图像
//         cv::imshow("QR Detection Result", image);
//         cv::waitKey(1); // 必要！用于刷新OpenCV窗口
        
//     } catch (cv_bridge::Exception& e) {
//         ROS_ERROR("图像格式转换失败: %s", e.what());
//     }
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "image_subscriber");
//     ros::NodeHandle nh;
    
//     // 创建image_transport对象
//     image_transport::ImageTransport it(nh);
    
//     // 订阅图像话题（话题名与发布者一致）
//     image_transport::Subscriber sub = it.subscribe(
//         "/qr_detector/debug_image",  // 话题名称
//         1,                           // 队列大小
//         imageCallback                // 回调函数
//     );
    
//     // 创建OpenCV窗口
//     cv::namedWindow("QR Detection Result", cv::WINDOW_AUTOSIZE);
    
//     // ROS事件循环
//     ros::spin();
    
//     // 退出时销毁窗口
//     cv::destroyAllWindows();
//     return 0;
// }