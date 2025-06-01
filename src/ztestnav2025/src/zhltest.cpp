#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include "ztestnav2025/turn_detect.h"


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"zhltest");
    ros::NodeHandle nh;

    MecanumController mecanumController(nh);

    std::vector<int> a = {-1,-1,-1,-1,-1,-1};
    mecanumController.detect(a,-1);
    ROS_INFO("测试");
    mecanumController.turn_and_find(1,1,1,0.4);
    
    ros::spin();

    return 0;
}

