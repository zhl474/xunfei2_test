#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include<geometry_msgs/Twist.h>
#include <vector>
#include <numeric>

using namespace cv;

class PIDController {
private:
    double kp, ki, kd;
    double integral;
    double error;
    ros::Time prev_time;
    int stop_flag = 1;
    int halfWidth = 340;
    int half = halfWidth;
    int mid_output = half;
    double prev_error;
    
public:
    PIDController(double p, double i, double d) : 
        kp(p), ki(i), kd(d), integral(0), error(0) {}
    
    void reset() {
        integral = 0;
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

    void mid(Mat &mask, int &error, int &should_stop) {


        for (int y = 480 - 1; y >= 0; --y) {
            int left = 0, right = 640;
            
            // 处理左半部分
                    // 修改后的左边界检测逻辑
        Mat leftPart;
        int left_found = 0;  // 标记是否找到边界

        // 优先检查左1/4位置 (half/2)
        int quarter_left = half / 2;
        for (int x = quarter_left; x >= max(0, quarter_left - 10); x--) {  // 检查左1/4附近区域
            if (mask.at<uchar>(y, x) == 255 && 
                mask.at<uchar>(y, x-1) == 0 && 
                mask.at<uchar>(y, x-2) == 0) {  // 白黑黑模式
                left = x;
                left_found = 1;
                break;
            }
        }

        // 如果左1/4没找到，再从中线向左遍历
        if (!left_found) {
            for (int x = half - 1; x >= 0; x--) {
                if (mask.at<uchar>(y, x) == 255 && 
                    mask.at<uchar>(y, x-1) == 0 && 
                    mask.at<uchar>(y, x-2) == 0) {  // 白黑黑模式
                    left = x;
                    left_found = 1;
                    break;
                }
            }
        }

        // 如果还是没找到边界，使用默认值
        if (!left_found) {
            left = max(0, half - halfWidth);
        }

        // 右边界检测逻辑（对称处理）
        Mat rightPart;
        int right_found = 0;

        // 优先检查右1/4位置 (half + quarter_left)
        int quarter_right = half + quarter_left;
        for (int x = quarter_right; x <= min(mask.cols-1, quarter_right + 10); x++) {
            if (mask.at<uchar>(y, x) == 255 && 
                mask.at<uchar>(y, x+1) == 0 && 
                mask.at<uchar>(y, x+2) == 0) {  // 黑白白模式
                right = x;
                right_found = 1;
                break;
            }
        }

        // 如果右1/4没找到，再从中线向右遍历
        if (!right_found) {
            for (int x = half; x < mask.cols; x++) {
                if (mask.at<uchar>(y, x) == 255 && 
                    mask.at<uchar>(y, x+1) == 0 && 
                    mask.at<uchar>(y, x+2) == 0) {  // 黑白白模式
                    right = x;
                    right_found = 1;
                    break;
                }
            }
        }

        // 如果还是没找到边界，使用默认值
        if (!right_found) {
            right = min(mask.cols, half + halfWidth);
        }


                    // 在特定行检查是否需要停车
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
                    mask.at<uchar>(y, mid) = 255;

                    if (y == 235) {
                        mid_output = mid;
                    }
                }
                error = 320 - mid_output;
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
    cap.open("/dev/video0", apiID);
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

 
        int error; // 中线偏差
        int should_stop = 0; // 停止标志

        pid_controller.mid(processed, error, should_stop);

        if (should_stop) {
            // 处理停车逻辑
        }
        // // 后续处理保持不变
        // 计算中线偏差

                  
            twist_angular_z = pid_controller.compute(error);
            twist_linear_x = 0.1;
            
            if(fabs(error) > 1.0) {
                twist_linear_x = 0.05;
            }
         else {
            ROS_WARN_THROTTLE(1, "Line not found!");
            pid_controller.reset();
            twist_linear_x = 0.0;
            twist_angular_z = -0.1;
        }
        
        // 发布控制指令
        // geometry_msgs::Twist twist;
        // twist.linear.x = min(0.2,twist_linear_x);
        // twist.angular.z = min(0.2,twist_linear_x);
        // cmd_pub.publish(twist);
        
        loop_rate.sleep();
    }

    // 正确释放资源（循环结束后执行）
    cap.release();
    destroyAllWindows();
    return 0;
}
