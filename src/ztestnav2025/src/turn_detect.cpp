// #include "ros/ros.h"
// #include "ros_nanodet/detect_result_srv.h"
// #include "ztestnav2025/getpose_server.h"

// #include <boost/algorithm/string/join.hpp>

// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/TransformStamped.h>

// #include <tf2_ros/transform_listener.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2/utils.h>
// #include <tf2/LinearMath/Quaternion.h>

// #include <cmath>

// #include "dynamic_reconfigure/server.h"
// #include "ztestnav2025/drConfig.h"

#include "ztestnav2025/turn_detect.h"

template<typename T>
T clamp(T value, T low, T high) {
    return (value < low) ? low : (value > high) ? high : value;
}


MecanumController::MecanumController(ros::NodeHandle& nh) : 
    nh_(nh),
    cmd_pub_(nh.advertise<geometry_msgs::Twist>("cmd_vel", 10)),
    detect_client_(nh.serviceClient<ros_nanodet::detect_result_srv>("nanodet_detect")),
    getpose_client_(nh.serviceClient<ztestnav2025::getpose_server>("getpose_server")),
    set_speed_client_(nh.serviceClient<ztestnav2025::getpose_server>("set_speed")),
    timer(nh.createTimer(ros::Duration(10.0), &MecanumController::timerCallback, this, false, false))
{
    if (!detect_client_.waitForExistence()) {
        ROS_FATAL("检测服务 nanodet_detect 不可用！");
        throw std::runtime_error("Service detect_result not found");
    }
    if (!getpose_client_.waitForExistence()) {
        ROS_FATAL("获取坐标服务 getpose_result 不可用！");
        throw std::runtime_error("Service getpose_result not found");
    }
    if (!set_speed_client_.waitForExistence()) {
        ROS_FATAL("运动控制服务 setspeed 不可用！");
        throw std::runtime_error("Service setspeed not found");
    }
    server_.setCallback(boost::bind(&MecanumController::PID_change, this, _1, _2));
}

void MecanumController::detect(std::vector<int>& result, int object_num){//封装目标检测功能
    start_detect_.request.detect_start = object_num;//要先传个2把摄像头打开
    bool flag = detect_client_.call(start_detect_);
    if (flag){
        result[0] = start_detect_.response.x0;result[1] = start_detect_.response.y0;result[2] = start_detect_.response.x1;result[3] = start_detect_.response.y1;result[4] = start_detect_.response.class_name;
        // ROS_INFO("结果：%d,%d,%d,%d,%d",start_detect_.response.class_name,start_detect_.response.x0,start_detect_.response.y0,start_detect_.response.x1,start_detect_.response.y1);
    }
    else{
        ROS_WARN("目标检测失败");
        return ;
    }
}

void MecanumController::cap_close(){
    start_detect_.request.detect_start = -2;
    bool flag = detect_client_.call(start_detect_);
    if (flag){
        ROS_INFO("目标检测摄像头已关闭");
    }
    else{
        ROS_WARN("请求处理失败....");
        return ;
    }
}

void MecanumController::rotateCircle(double rotate,int direction, double angular_speed) {//控制小车运动，rotate是弧度,direction逆时针是正向
    geometry_msgs::Twist twist;
    ros::Rate rate(20);     // 控制频率20Hz

    std::vector<float> position_array = getCurrentPose();
    double start = position_array[2];
    double target = start+rotate;
    if(target>=3.14){
        target = target - 6.28;//最大就3.14，超过3.14变成-3.14
    }
    // ROS_INFO("起点和目标:%f,%f",start,target);
    while (ros::ok()) {
        // 获取当前姿态
        std::vector<float> now_yaw = getCurrentPose();
        if (now_yaw.empty()) continue;

        // 计算旋转控制量（参考网页2的PWM控制原理）
        double yaw = now_yaw[2];
        if (fabs(yaw - target) <= angle_error_) {
            break;
        }

        // 发送运动指令（参考网页1的速度发布逻辑）
        twist.angular.z = angular_speed * direction;
        cmd_pub_.publish(twist);
        rate.sleep();
    }
}

int MecanumController::turn_and_find(double x,int y,int z,double angular_speed){//原地旋转小车x度，执行y次目标检测,寻找z号目标
    std::vector<int> result = {-1,-1,-1,-1,-1,-1};

        double integral = 0, prev_error = 0;
        // ros::Rate rate(20);     // 控制频率20Hz
        geometry_msgs::Twist twist;
        timer.start();
        while(ros::ok()&&!exit_flag){
            ros::spinOnce();
            detect(result, z);     // 持续检测目标
            if(result[4] != z){
                set_speed_.request.getpose_start= static_cast<int>(angular_speed*100);
                set_speed_client_.call(set_speed_);
                integral = 0;
                continue;
            }  // 目标丢失则旋转寻找目标
            
            // 计算中心点偏差（误差输入）
            int center_x = (result[0]+result[2])/2;
            // 退出条件：误差<7像素
            if(std::abs(center_x - img_width/2) < 7){
                ROS_INFO("已经对准");
                integral = 0;
                result[5];
            } 
            double error = (img_width/2.0 - center_x)/100; 
            
            // 离散PID计算
            integral += error * 0.05;       // dt=1/20≈0.05
            double derivative = (error - prev_error)/0.05;
            double output = Kp_*error + Ki_*integral + Kd_*derivative;
            output = clamp(output, -1.0, 1.0);
            // ROS_INFO("速度发布:%f",output);
            
            // 执行旋转（限制输出范围）
            set_speed_.request.getpose_start = static_cast<int>(angular_speed*100*output);
            getpose_client_.call(set_speed_);
            
            prev_error = error;
        }
    set_speed_.request.getpose_start = 0;
    getpose_client_.call(set_speed_);
    return result[5];
}

    //解析动态参数
void MecanumController::PID_change(ztestnav2025::drConfig &config, uint32_t level){
    Kp_ = config.Kp; 
    Ki_ = config.Ki;
    Kd_ = config.Kd;
    pid_change_flag = 1;
    ROS_INFO("PID参数修改,P:%f,I:%f,D:%f",config.Kp,config.Ki,config.Kd);
}

std::vector<float> MecanumController::getCurrentPose(){
    start_get_pose_.request.getpose_start= 1;
    if (getpose_client_.call(start_get_pose_)){
        // ROS_INFO("请求正常处理,响应结果长度%zu",start_get_pose_.response.pose_at.size());
        std::vector<float> pose_array = start_get_pose_.response.pose_at;
        return pose_array;
    }
    else{
        ROS_ERROR("请求处理失败....");
        return {};
    }
}

void MecanumController::timerCallback(const ros::TimerEvent&) {
    exit_flag = true;
    timer.stop(); // 停止定时器
}

// int main(int argc, char *argv[])
// {
//     setlocale(LC_ALL,"");
//     ros::init(argc,argv,"move_func_test");
//     ros::NodeHandle nh;
//     MecanumController controller(nh);

//     ROS_INFO("视觉");
//     ros::spin();
// }