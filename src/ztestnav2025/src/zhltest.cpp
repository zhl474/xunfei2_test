#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "ztestnav2025/getpose_server.h"
#include "ztestnav2025/turn_detect.h"
#include "ztestnav2025/lidar_process.h"
#include "line_follow/line_follow.h"
#include "ztestnav2025/traffic_light.h"
#include "qr_01/qr_srv.h"
#include "communication/msg_1.h"
#include "communication/msg_2.h"

#include <cmath>


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"zhltest");
    ros::NodeHandle nh;
    ros::Rate control_rate(20);
    ROS_INFO("测试");
    while(ros::ok()){
        int result = detectTrafficLightStatus();
        ROS_INFO("%d",result);
        control_rate.sleep();
    }

    return 0;
}

// int main(int argc, char *argv[])
// {
//     setlocale(LC_ALL,"");
//     ros::init(argc,argv,"zhltest");
//     ros::NodeHandle nh;

//     MecanumController mecanumController(nh);

//     std::vector<int> a = {-1,-1,-1,-1,-1,-1};
//     mecanumController.detect(a,-1);
// restart:
//     mecanumController.turn_and_find(1,1,1,0.4);
    
//     while(ros::ok()){
//         ros::spinOnce();
//         if(mecanumController.pid_change_flag == 1){
//             mecanumController.pid_change_flag = 0;
//             goto restart;
//         }
//     }
//     ros::spin();

//     return 0;
// }


