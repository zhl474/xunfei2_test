///move_base/local_costmap/obstacle_layer/set_parameters
#include <ros/ros.h>
#include <ros/master.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/client.h>
#include <teb_local_planner/TebLocalPlannerReconfigureConfig.h>

int main(int argc,char ** argv){
    ros::init(argc,argv,"dynamic_reconfigure");
    ros::ServiceClient client;
    ros::NodeHandle n;
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::Config config;
    dynamic_reconfigure::DoubleParameter double_param;
    client = n.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/TebLocalPlannerROS/set_parameters");
    double_param.name = "max_global_plan_lookahead_dist" ;
    double_param.value = 0.6;
    config.doubles.push_back(double_param);
    srv.request.config = config;
    if(client.call(srv)){
        ROS_INFO("SUCCESS!!!!0.6");
    }
    else{
        ROS_INFO("UNSECCESS!!!");
    }

    ros::ServiceClient client1;
    dynamic_reconfigure::Reconfigure srv1;
    dynamic_reconfigure::Config config1;
    dynamic_reconfigure::DoubleParameter double_param1;

    ros::Duration(1).sleep();
    client = n.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/TebLocalPlannerROS/set_parameters");
    double_param1.name = "max_global_plan_lookahead_dist";
    double_param1.value = 0.8;
    config1.doubles.push_back(double_param1);
    srv1.request.config = config1;
    if(client.call(srv1)){
        ROS_INFO("SUCCESS!!!0.8");
    }
    else{
        ROS_INFO("UNSECCESS!!");
    }
}