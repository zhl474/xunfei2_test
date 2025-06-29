#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tensor_rt/Messages.h"

int room_a = 0;//存储三个房间的类型
int room_b = 0;
int room_c = 0;
int processDetectionResult(const tensor_rt::Messages::Response& resp, int goal_num) // 进入房间后处理检测结果（&表示resp是该类型的引用（而不是拷贝），直接引用原数据）
{
    if(resp.result.empty()){
        ROS_INFO("Goal_%d: No objects detected", goal_num);
        return -1;  // 使用-1表示无检测结果
    }
    
    int max_confidence = 0;
    int detected_class = -1;
    
    // 遍历所有检测结果（每6个元素为一个检测结果）
    for(size_t i = 0; i < resp.result.size(); i += 6){
        int current_conf = resp.result[i+5];  // 置信度位于第6个元素
        int current_class = resp.result[i+4]; // 类别位于第5个元素
        
        ROS_INFO("Goal_%d detected: Class-%d (Confidence: %d%%)", 
                goal_num, current_class, current_conf);
                
        // 比较置信值
        if(current_conf > max_confidence) {
            max_confidence = current_conf;
            detected_class = current_class;
        }
    }
    
    if(detected_class != -1) {
        ROS_INFO("Goal_%d selected class: %d with confidence: %d%%", 
                goal_num, detected_class, max_confidence);
    }
    
    return detected_class;
}


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char* argv[])//goal 1 3 5 正对着三个房间，在这三个点请求tensor_rt服务
{
    ros::init(argc, argv, "nav_client_1");
    ros::NodeHandle nh;
    MoveBaseClient ac("move_base", true);
    ros::ServiceClient client = nh.serviceClient<tensor_rt::Messages>("tensorRT_detect");// 创建服务客户端
    client.waitForExistence(ros::Duration(5));  // 等待服务就绪
    int i = 0;
    // 等待导航启动
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    //坐标系为map
    goal.target_pose.header.frame_id = "map";
    // 时间戳
    goal.target_pose.header.stamp = ros::Time::now();
    // 目标位置
    goal.target_pose.pose.position.x = 4.03508996963501;
    goal.target_pose.pose.position.y = 1.2556221961975098;
    goal.target_pose.pose.position.z = 0.0; 
    // 目标方向
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.7071;
    goal.target_pose.pose.orientation.w = 0.7071;

    ROS_INFO("Sending goal_1");
    // 发送目标位置
    ac.sendGoal(goal);
    // 等待结果
    ac.waitForResult();
    // 判断结果
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Goal_1 reached");
        tensor_rt::Messages trt;// 分别单独创建服务请求，生命周期为函数体内，无需清除
        if(client.call(trt)) 
        {
            room_a = processDetectionResult(trt.response, 1);
        }
        i = 2;
    }
    else
    {
        ROS_INFO("The base failed to move to goal_1");
    }
    
    if(i == 2)
    {
        // 目标位置
        goal.target_pose.pose.position.x = 2.727267026901245;
        goal.target_pose.pose.position.y = 1.514411392211914;
        goal.target_pose.pose.position.z = 0.0; 
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.866;
        goal.target_pose.pose.orientation.w = 0.5;

        ROS_INFO("Sending goal_3");
        // 发送目标位置
        ac.sendGoal(goal);
        // 等待结果
        ac.waitForResult();
        // 判断结果
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal_3 reached");
            tensor_rt::Messages trt;// 分别单独创建服务请求，生命周期为函数体内，无需清除
            if(client.call(trt)) 
            {
                room_b = processDetectionResult(trt.response, 3);
            }
            i = 4;
        }
        else
        {
            ROS_INFO("The base failed to move to goal_3");
        }
    }
    
    if(i == 4)
    {
        // 目标位置
        goal.target_pose.pose.position.x = 0.7367432832717896;
        goal.target_pose.pose.position.y = 1.6391351699829102;
        goal.target_pose.pose.position.z = 0.0; 
        // 目标方向
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.7071;
        goal.target_pose.pose.orientation.w = 0.7071;

        ROS_INFO("Sending goal_5");
        // 发送目标位置
        ac.sendGoal(goal);
        // 等待结果
        ac.waitForResult();
        // 判断结果
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal_5 reached");
            tensor_rt::Messages trt;// 分别单独创建服务请求，生命周期为函数体内，无需清除
            if(client.call(trt)) 
            {
                room_c = processDetectionResult(trt.response, 5);
            }
            i = 8;
        }
        else
        {
            ROS_INFO("The base failed to move to goal_5");
        }
    }
    
    
    
    
    if(i == 8)
    {
        // 目标位置
        goal.target_pose.pose.position.x = 0;
        goal.target_pose.pose.position.y = 0;
        goal.target_pose.pose.position.z = 0.0; 
            // 目标方向
        goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0;
        goal.target_pose.pose.orientation.w = 1;

        ROS_INFO("Sending goal_9");
            // 发送目标位置
        ac.sendGoal(goal);
            // 等待结果
        ac.waitForResult();
            // 判断结果
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal_9 reached");
            i = 10;
        }
        else
        {
            ROS_INFO("The base failed to move to goal_9");
        }
    }
    ROS_INFO("room_a: %d, room_b: %d, room_c: %d", room_a, room_b, room_c);
    

    return 0;
}
 