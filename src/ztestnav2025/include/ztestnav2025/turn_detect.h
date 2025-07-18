#ifndef TURN_DETECT_H
#define TURN_DETECT_H

#include "ros/ros.h"
#include "ros_nanodet/detect_result_srv.h"
#include "ztestnav2025/getpose_server.h"
#include "ztestnav2025/set_speed.h"
#include "ztestnav2025/lidar_process.h"
#include "ztestnav2025/traffic_light.h"

#include <boost/algorithm/string/join.hpp>

#include <geometry_msgs/Twist.h>

#include "dynamic_reconfigure/server.h"
#include "ztestnav2025/drConfig.h"

class MecanumController {
public:
    MecanumController(ros::NodeHandle& nh);

    void detect(std::vector<int>& result, int object_num);
    void rotateCircle(double rotate,double angular_speed=0.2); //控制小车运动，rotate是弧度,direction逆时针是正向
    int turn_and_find(double find_time,int y,int z, double angular_speed);//原地旋转小车time秒，执行y次目标检测,寻找z号目标
    void PID_change(ztestnav2025::drConfig &config, uint32_t level);    //解析动态参数
    std::vector<float> getCurrentPose();
    void cap_close();
    bool forward(int z,double forward_speed);
    bool adjust(int z,double adjust_speed);

    bool pid_change_flag=0;

    ros::Publisher cmd_pub_;

private:
    // ROS通信接口
    ros::NodeHandle nh_;

    // 运动学参数
    double angle_error_ = 0.034;   //容忍2度偏差
    double Kp_ = 0.5;  // 先调这个参数
    double Ki_ = 0.5;  // 最后调积分项
    double Kd_ = 0.1;  // 中间调微分项

    ros::ServiceClient detect_client_;
    ros_nanodet::detect_result_srv start_detect_;//目标检测客户端
    std::vector<int> result;
    
    ros::ServiceClient getpose_client_;
    ztestnav2025::getpose_server start_get_pose_;

    ros::ServiceClient set_speed_client_;
    ztestnav2025::set_speed set_speed_;

    ros::ServiceClient adjust_client_;
    ztestnav2025::lidar_process board_slope;//lidarprocess客户端

    dynamic_reconfigure::Server<ztestnav2025::drConfig> server_;//动态参数

    ros::Timer timer;
    ros::Time start_time_;
    ros::Time now_;
    bool exit_flag = false;
    void timerCallback(const ros::TimerEvent&);

    int img_width = 640;
    int img_height = 480;
    int focal_distance = 2.8;//单位毫米
    double width_per_pixel = 10.7118/img_width;
    double height_per_pixel = 3.7066/img_height;

};

#endif