// #include "ros/ros.h"
// #include <actionlib/client/simple_action_client.h>
// #include <move_base_msgs/MoveBaseAction.h>

// #include <tf2/LinearMath/Quaternion.h>
// #include <geometry_msgs/PoseStamped.h>

// #include "ztestnav2025/getpose_server.h"
// #include "ztestnav2025/turn_detect.h"
// #include "ztestnav2025/lidar_process.h"
// #include "line_follow/line_follow.h"
// #include "qr_01/qr_srv.h"
// #include "communication/msg_1.h"
// #include "communication/msg_2.h"

// #include <cmath>


// int main(int argc, char *argv[]){//请求二维码识别服务
//             ros::Duration(1).sleep();
//             what_qr.request.qr_start = 1;
//             while(ros::ok()){
//                 if (!client_qr.call(what_qr)){
//                     ROS_INFO("没有请求到服务");
//                 }
//                 board_class = what_qr.response.qr_result;
//                 ROS_INFO("二维码结果:%d",board_class);
//                 if (board_class>0){
                    
//                     ROS_INFO("二维码结果:%d",what_qr.response.qr_result);
//                     break;
//                 }
//                 else{
//                     ROS_ERROR("请求二维码失败");
//                 }
//             }
// }