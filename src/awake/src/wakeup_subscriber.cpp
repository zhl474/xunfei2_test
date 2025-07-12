#include "ros/ros.h"
#include <std_msgs/Int8.h>
#include <chrono>
#include <thread>

// 模拟语音唤醒信号发布节点
int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "wakeup_subscriber");
    ros::NodeHandle nh;
    
    // 创建话题发布者
    ros::Publisher awake_pub = nh.advertise<std_msgs::Int8>("/awake_flag", 10);
    
    ROS_INFO("语音唤醒测试节点启动，准备发布唤醒信号...");
    
    // 测试流程：
    // 1. 先发布awake_flag=0（未唤醒状态）
    // 2. 等待一段时间后发布awake_flag=1（唤醒状态）
    std_msgs::Int8 awake_msg;
    
    // 阶段1：未唤醒状态（awake_flag=0）
    awake_msg.data = 0;
    ROS_INFO("发布未唤醒信号 (awake_flag=0)");
    for (int i = 0; i < 5; i++) {
        awake_pub.publish(awake_msg);
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // 阶段2：模拟语音唤醒，发布awake_flag=1
    awake_msg.data = 1;
    ROS_INFO("发布唤醒信号 (awake_flag=1)");
    for (int i = 0; i < 3; i++) {
        awake_pub.publish(awake_msg);
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // 阶段3：唤醒后保持awake_flag=1
    ROS_INFO("保持唤醒状态 (awake_flag=1)");
    while (ros::ok()) {
        awake_pub.publish(awake_msg);
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    return 0;
}