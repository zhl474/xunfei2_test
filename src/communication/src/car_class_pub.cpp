

#include "ros/ros.h"

#include "communication/msg_1.h"

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");

    // 1. 初始化 ROS 节点
    ros::init(argc, argv, "class_publisher");

    // 2. 创建 ROS 句柄
    ros::NodeHandle nh;

    // 3. 创建发布者对象，话题名称为 send_class
    ros::Publisher pub = nh.advertise<communication::msg_1>("send_class", 10);

    // 4. 创建自定义消息对象
    communication::msg_1 target_msg;
    uint16_t target = 0; // 初始目标类别

    // 设置发布频率
    ros::Rate rate(1);

    while (ros::ok()) {
        // 设置消息内容
        target_msg.target_class = target;


        // 发布消息
        pub.publish(target_msg);

        // 打印日志信息
        ROS_INFO("发布目标类别: %d", target_msg.target_class);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
