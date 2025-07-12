
#include "ros/ros.h"
#include "detect_client/Messages.h"


int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"detect_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<detect_client::Messages>("tensorRT_detect");
    ros::service::waitForService("tensorRT_detect");
    detect_client::Messages image;
    image.request.original_images;

    bool flag = client.call(image);

    if (flag)
    {
        ROS_INFO("请求正常处理,响应结果");
    }
    else
    {
        ROS_ERROR("请求处理失败....");
        return 1;
    }

    return 0;
}