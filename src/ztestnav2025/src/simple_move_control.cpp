#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "ztestnav2025/getpose_server.h"//套用定位请求的文件

float speed;
bool start = false;
// 速度设置服务回调
bool setSpeedCallback(ztestnav2025::getpose_server::Request &req,ztestnav2025::getpose_server::Response &resp)
{
    if (req.getpose_start == 0) {
        ROS_INFO("运动控制节点停止");
        start=false;
    }
    else {
        // 业务逻辑：检测成功时设置为正向速度，失败时为0
        speed = req.getpose_start / 100.0;
        start = true;
        // ROS_INFO("运动控制节点运行中");
    }
    resp.pose_at.resize(1);
    resp.pose_at[0] = 1.0;
    return true;
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "simple_move_control");
    ros::NodeHandle nh;
    
    // 创建服务服务器
    ros::ServiceServer speed_service = nh.advertiseService(
        "set_speed", setSpeedCallback);
    
    // 创建速度发布器
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>(
        "/cmd_vel", 1);
    
    // 初始化控制循环
    ros::Rate control_rate(20); // 严格20Hz频率
    geometry_msgs::Twist twist_msg;
    twist_msg.angular.z = 0;
    
    ROS_INFO("运动控制节点已启动 (20Hz 控制循环)");
    
    while (ros::ok()) {
        // 1. 处理回调获取当前速度值
        ros::spinOnce();
        twist_msg.angular.z = speed;
        if(start) cmd_pub.publish(twist_msg);
        control_rate.sleep();
    }

    ROS_INFO("节点关闭，发送停止指令");
    return 0;
}