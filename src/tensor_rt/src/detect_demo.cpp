#include <ros/ros.h>
#include "tensor_rt/Messages.h"

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "detect_test");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<tensor_rt::Messages>("tensorRT_detect");
    client.waitForExistence();
    tensor_rt::Messages trt;
    trt.request.object = 0;

    ros::Rate r(50);
    while (ros::ok()){
        bool flag = client.call(trt);
        if (flag)
        {
            if(trt.response.result.empty()){
                ROS_INFO("未检测到目标");
            }
            else{
                ROS_INFO("位置:%d,%d,%d,%d,类别:%d,置信度:%d",trt.response.result[0],trt.response.result[1],trt.response.result[2],trt.response.result[3],trt.response.result[4],trt.response.result[5]);
            }
        }
        else
        {
            ROS_ERROR("请求处理失败");
            return 1;
        }
        r.sleep();
    }
    
    return 0;
}
