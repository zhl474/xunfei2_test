#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

class ImageConverter {
public:
    ImageConverter() : it_(nh_) {
        // 订阅摄像头数据（根据实际设备修改话题名称）
        image_sub_ = it_.subscribe(
            "/camera/image_raw", 
            1, 
            &ImageConverter::imageCallback, 
            this
        );
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            // 核心转换代码 --------------------------------------------------
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
                msg, 
                sensor_msgs::image_encodings::BGR8
            );
            cv::Mat frame = cv_ptr->image;
            // ------------------------------------------------------------
            
            /* 此处可添加图像处理代码
            cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
            cv::imshow("Preview", frame);
            cv::waitKey(1);
            */

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge转换失败: %s", e.what());
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    ros::spin();
    return 0;
}