

#include "ros/ros.h"
#include "communication/msg_2.h"

void doPerson(const communication::msg_2::ConstPtr& detect){
    ROS_INFO("房间编号：%d,检测类型：%d", detect->room, detect->detected_class);
}

int main(int argc, char *argv[])
{   
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"webbsocket_sub_node");
    //2.创建 ROS 句柄
    ros::NodeHandle nh;
    //3.创建订阅对象
    ros::Subscriber sub = nh.subscribe<communication::msg_2>("send_room_class",10,doPerson);

    //4.回调函数中处理 person

    //5.ros::spin();
    ros::spin();    
    return 0;
}