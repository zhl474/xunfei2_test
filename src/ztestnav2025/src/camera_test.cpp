#include "ros/ros.h"
#include "ros_nanodet/detect_result_srv.h"
#include "ztestnav2025/getpose_server.h"

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <boost/algorithm/string/join.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

#include "dynamic_reconfigure/server.h"
#include "ztestnav2025/drConfig.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

template<typename T>
T clamp(T value, T low, T high) {
    return (value < low) ? low : (value > high) ? high : value;
}

class MecanumController {
public:
    MecanumController(ros::NodeHandle& nh) : 
        nh_(nh),
        cmd_pub_(nh.advertise<geometry_msgs::Twist>("cmd_vel", 10)),
        detect_client_(nh.serviceClient<ros_nanodet::detect_result_srv>("detect_result")),
    {
        if (!detect_client_.waitForExistence()) {
            ROS_FATAL("检测服务 detect_result 不可用！");
            throw std::runtime_error("Service detect_result not found");
        }
        server_.setCallback(boost::bind(&MecanumController::PID_change, this, _1, _2));
    }

    void detect(std::vector<int>& result){//封装目标检测功能
        start_detect_.request.detect_start = 1;
        bool flag = detect_client_.call(start_detect_);
        if (flag)
        {
            result[0] = start_detect_.response.x0;result[1] = start_detect_.response.y0;result[2] = start_detect_.response.x1;result[3] = start_detect_.response.y1;result[4] = start_detect_.response.class_name;
            ROS_INFO("结果：%d,%d,%d,%d,%d",start_detect_.response.class_name,start_detect_.response.x0,start_detect_.response.x1,start_detect_.response.y0,start_detect_.response.y1);
        }
        else
        {
            ROS_WARN("请求处理失败....");
            return ;
        }
    }

    void rotateCircle(double rotate,int direction) {//控制小车运动，rotate是弧度,direction逆时针是正向
        geometry_msgs::Twist twist;
        ros::Rate rate(20);     // 控制频率20Hz
        auto start_opt = getCurrentPose();
        if (!start_opt) ROS_INFO("坐标获取失败");
        double start = getYawFromTransform(*start_opt);
        double target = start+rotate;
        if(target>=3.14){
            target = target - 6.28;//最大就3.14，超过3.14变成-3.14
        }
        ROS_INFO("起点和目标:%f,%f",start,target);
        while (ros::ok()) {
            // 获取当前姿态（参考网页7的TF查询方法）
            auto now_yaw = getCurrentPose();
            if (!now_yaw) continue;

            // 计算旋转控制量（参考网页2的PWM控制原理）
            double yaw = getYawFromTransform(*now_yaw);
            if (fabs(yaw - target) <= angle_error_) {
                break;
            }

            // 发送运动指令（参考网页1的速度发布逻辑）
            twist.angular.z = angular_speed_ * direction;
            cmd_pub_.publish(twist);
            rate.sleep();
        }
    }

    void turn_and_find(double x,int y,int z){//原地旋转小车x度，执行y次目标检测,寻找z号目标
        std::vector<int> result = {-1,-1,-1,-1,-1};
            
            double integral = 0, prev_error = 0;
            ros::Rate rate(20);     // 控制频率20Hz
            geometry_msgs::Twist twist;
            while(ros::ok()){
                detect(result);     // 持续检测目标
                if(result[4] != z){
                    twist.angular.z = angular_speed_;
                    cmd_pub_.publish(twist);
                    integral = 0;
                    continue;
                }  // 目标丢失则旋转寻找目标
                
                // 计算中心点偏差（误差输入）
                int center_x = (result[0]+result[2])/2;
                // 退出条件：误差<7像素
                if(std::abs(center_x - img_width/2) < 7){
                    ROS_INFO("center,wid:%d,%d",center_x,img_width);
                    integral = 0;
                    break;
                } 
                double error = (img_width/2.0 - center_x)/10; 
                
                // 离散PID计算
                integral += error * 0.05;       // dt=1/20≈0.05
                double derivative = (error - prev_error)/0.05;
                double output = Kp_*error + Ki_*integral + Kd_*derivative;
                output = clamp(output, -1.0, 1.0);
                ROS_INFO("速度发布:%f",output);
                
                // 执行旋转（限制输出范围）
                twist.angular.z = angular_speed_ * output;
                cmd_pub_.publish(twist);
                
                prev_error = error;
            }
    }

    //解析动态参数
    void PID_change(ztestnav2025::drConfig &config, uint32_t level){
        Kp_ = config.Kp; 
        Ki_ = config.Ki;
        Kd_ = config.Kd;
        ROS_INFO("PID参数修改,P:%f,I:%f,D:%f",config.Kp,config.Ki,config.Kd);
    }


private:
    
    // ROS通信接口
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::ServiceServer getpose;
    
    // 运动学参数
    double angular_speed_ = 0.2;   // 默认角速度(rad/s)
    double angle_error_ = 0.034;   //容忍2度偏差
    double Kp_ = 0.5;  // 先调这个参数
    double Ki_ = 0.1;  // 最后调积分项
    double Kd_ = 0.1;  // 中间调微分项

    ros::ServiceClient detect_client_;
    ros_nanodet::detect_result_srv start_detect_;//目标检测客户端
    dynamic_reconfigure::Server<ztestnav2025::drConfig> server_;//动态参数

    int img_width = 640;
    int img_height = 480;
    int focal_distance = 2.8;//单位毫米
    double width_per_pixel = 10.7118/img_width;
    double height_per_pixel = 3.7066/img_height;

};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"move_func_test");
    ros::NodeHandle nh;
    MecanumController controller(nh);

    // controller.turn_and_find(360,8,0);
    ROS_INFO("done");
    ros::spin();
}