#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "wakeup_publisher");
    
    // 创建节点句柄
    ros::NodeHandle nh;
    
    // 创建发布者，发布到/wakeup_status话题，队列长度10
    ros::Publisher wakeup_pub = nh.advertise<std_msgs::String>("wakeup_status", 10);
    
    // 设置发布频率 (10Hz)
    ros::Rate loop_rate(10);
    
    ROS_INFO("唤醒状态发布者已启动，等待唤醒事件...");
    
    while (ros::ok())
    {
        // 这里应替换为实际的唤醒检测逻辑
        std::string input;
        std::cout << "输入'y'模拟唤醒事件，输入其他退出: ";
        std::getline(std::cin, input);
        
        if (input == "y")
        {
            // 创建消息并发布
            std_msgs::String msg;
            msg.data = "WAKEUP_SUCCESS";
            wakeup_pub.publish(msg);
            ROS_INFO("已发布唤醒成功消息");
            
            // 休眠1秒避免重复发布
            ros::Duration(1.0).sleep();
        }
        else
        {
            break;
        }
        
        // 处理回调函数
        ros::spinOnce();
        
        // 按照设定频率休眠
        loop_rate.sleep();
    }
    
    return 0;
}
