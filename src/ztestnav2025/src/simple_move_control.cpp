#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "ztestnav2025/set_speed.h"//套用定位请求的文件

geometry_msgs::Twist twist_msg;
bool start = false;
// 速度设置服务回调
bool setSpeedCallback(ztestnav2025::set_speed::Request &req,ztestnav2025::set_speed::Response &resp)
{
    if (req.work == false) {
        ROS_INFO("运动控制节点停止");
        start=false;
    }
    else {
        // 业务逻辑：检测成功时设置为正向速度，失败时为0
        twist_msg = req.target_twist;
        start = true;
        // ROS_INFO("运动控制节点运行中");
    }
    resp.success = true;
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
    
    ROS_INFO("运动控制节点已启动 (20Hz 控制循环)");
    
    while (ros::ok()) {
        // 1. 处理回调获取当前速度值
        ros::spinOnce();
        if(start) cmd_pub.publish(twist_msg);
        control_rate.sleep();
    }

    ROS_INFO("节点关闭，发送停止指令");
    return 0;
}