#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include<geometry_msgs/Twist.h>

using namespace cv;

class PIDController {
private:
    double kp, ki, kd;
    double integral;
    double prev_error;
    ros::Time prev_time;
    
public:
    PIDController(double p, double i, double d) : 
        kp(p), ki(i), kd(d), integral(0), prev_error(0) {}
    
    void reset() {
        integral = 0;
        prev_error = 0;
    }
    
    double compute(double error) {
        ros::Time now = ros::Time::now();
        double dt = (now - prev_time).toSec();
        prev_time = now;
        
        if (dt <= 0) dt = 0.01;
        
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        
        if (integral > 1.0) integral = 1.0;
        if (integral < -1.0) integral = -1.0;
        
        return kp * error + ki * integral + kd * derivative;
    }
};

double twist_linear_x, twist_angular_z;

bool use_binary_thresh = true;      // 是否使用二值化阈值
int threshold_value = 125;          // 阈值
int threshold_type = THRESH_BINARY; // 默认使用BINARY阈值

int main(int argc, char **argv){
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "follower_line_pid");
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Rate loop_rate(20);

    double p, i, d;
    PIDController pid_controller(p, i, d);
    
    VideoCapture cap;//直接通过opencv启动摄像头
    int apiID = CAP_V4L2;   
    cap.open("/dev/video0", apiID);
    if (!cap.isOpened()) {
    std::cerr << "ERROR! 摄像头无法打开" << std::endl;
    return -1;
    }
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    Mat frame;
    ROS_INFO("巡线摄像头启动");

    while(ros::ok()){
        cap.read(frame);
        if (frame.empty()) continue;
        Mat gray, processed;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        threshold(gray, processed, threshold_value, 255, threshold_type);
        imshow("Camera Feed", processed);
        waitKey(1);

        // 后续处理保持不变
        int h = frame.rows, w = frame.cols;
        int search_top = 5 * h / 6;
        int search_bot = search_top + 20;
        
        processed.rowRange(0, search_top).setTo(0);
        processed.rowRange(search_bot, h).setTo(0);

        Moments M = moments(processed);
        
        if(M.m00 > 0){
            int cx = int(round(M.m10 / M.m00));
            int cy = int(round(M.m01 / M.m00));
            
            ROS_INFO("Center: (%d, %d)", cx, cy);
            circle(frame, Point(cx, cy), 10, Scalar(255, 255, 255), -1);
            
            double error = (w / 2 - cx) / 100.0;
            twist_angular_z = pid_controller.compute(error);
            twist_linear_x = 0.1;
            
            if(fabs(error) > 1.0) {
                twist_linear_x = 0.05;
            }
        } 
        else {
            ROS_WARN("Line not found!");
            pid_controller.reset();
            twist_linear_x = 0.0;
            twist_angular_z = -0.1;
        }
            
        geometry_msgs::Twist twist;
        twist.linear.x = twist_linear_x;
        twist.angular.z = twist_angular_z;
        cmd_pub.publish(twist);
        loop_rate.sleep();
    }

    cap.release();
    destroyAllWindows();
    return 0;
}