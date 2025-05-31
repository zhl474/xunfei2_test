#include <ros/ros.h>
#include <std_msgs/String.h>

// 回调函数，处理接收到的唤醒状态消息
void wakeupCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("收到唤醒状态: %s", msg->data.c_str());
    
    // 根据唤醒状态执行相应操作
    if (msg->data == "WAKEUP_SUCCESS")
    {
        ROS_INFO("开始执行唤醒后的操作...");
        // 这里可以添加唤醒后要执行的代码
        // 例如启动语音识别、控制小车移动等
    }
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "wakeup_subscriber");
    
    // 创建节点句柄
    ros::NodeHandle nh;
    
    // 创建订阅者，订阅/wakeup_status话题
    ros::Subscriber sub = nh.subscribe("wakeup_status", 10, wakeupCallback);
    
    // 进入循环等待消息
    ROS_INFO("唤醒状态订阅者已启动，等待消息...");
    ros::spin();
    
    return 0;
}
