#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tensor_rt/Messages.h"
#include <iostream>
#include <string>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>

namespace beast = boost::beast;//简化nlohmann库中json的名称
namespace websocket = beast::websocket;//简化beast库中websocket的名称
using json = nlohmann::json;//将nlohmann的JSON库设为默认的JSON解析器

// 全局变量存储目标类别
uint16_t class_1 = -1;
//需要检测的目标类别
uint16_t target_1 = -1;
uint16_t target_2 = -1;
uint16_t target_3 = -1;

//解析传入的json数据
void handle_message(const std::string& message) {
    try {
        auto parsed = json::parse(message);
        
        if (parsed.contains("msg") && 
            parsed["msg"].contains("target_class")) {
            class_1 = parsed["msg"]["target_class"];
            ROS_INFO("[Class Updated] Current class: %d", class_1);
        }
    } catch (const json::exception& e) {
        ROS_ERROR("JSON解析错误: %s", e.what());
    }
}


uint16_t room_a = -1;//存储三个房间的类型
uint16_t room_b = -1;
uint16_t room_c = -1;
uint16_t target_room = 0;
uint16_t final_class = -1;
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
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "nav_client_2");
    ros::NodeHandle nh;
    MoveBaseClient ac("move_base", true);
    ros::ServiceClient client = nh.serviceClient<tensor_rt::Messages>("tensorRT_detect");// 创建服务客户端

    //此处中断，并接收来自小车的数据
    try {
        //配置连接参数
        std::string host = "192.168.43.39";
        std::string port = "9090";
        std::string target = "/";
        //初始化网络组件
        //初始化IO上下文
        boost::asio::io_context ioc;

        //创建解析器和连接
        boost::asio::ip::tcp::resolver resolver(ioc);
        websocket::stream<boost::asio::ip::tcp::socket> ws(ioc);

        //建立TCP连接
        auto const results = resolver.resolve(host, port);
        boost::asio::connect(ws.next_layer(), results.begin(), results.end());
        
        
        std::string host_with_port = host + ":" + port;//构造host头字段
        ws.handshake(host_with_port, target);//完成WebSocket握手

        //发送订阅消息
        json subscribe_msg = {
            {"op", "subscribe"},//op指定操作为订阅
            {"topic", "/send_class"},//订阅的话题
            {"type", "communication/msg_1"}   //此处自定义文件的路径必须为小车内的路径，而非本地路径
        };
        ws.write(boost::asio::buffer(subscribe_msg.dump()));//通过websocket发送序列化的信息

        // 持续接收消息
        ROS_INFO("WebSocket客户端已启动,等待消息...");
        for(;;) {
            beast::flat_buffer buffer;//用flat_buffer接收二进制数据
            ws.read(buffer);//将二进制转为字符串
            handle_message(beast::buffers_to_string(buffer.data()));//解析内容
            if (class_1 != -1)
            {
                break;
            }
        }

    } catch (const std::exception& e) {
        ROS_ERROR("运行时错误: %s", e.what());
        return 1;
    }

    //分析消息内容设置目标类型
    if (class_1 == 8 || class_1 == 0 || class_1 == 1)
    {
        target_1 = 8;
        target_2 = 0;
        target_3 = 1;
    }
    if (class_1 == 2 || class_1 == 3 || class_1 == 4)
    {
        target_1 = 2;
        target_2 = 3;
        target_3 = 4;
    }
    if (class_1 == 5 || class_1 == 6 || class_1 == 7)
    {
        target_1 = 5;
        target_2 = 6;
        target_3 = 7;
    }






    //继续开始导航
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
        goal.target_pose.pose.position.x = 0.9367432832717896;
        goal.target_pose.pose.position.y = 1.8391351699829102;
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


    //判断目标所在的房间并播报
    if (room_a == target_1 || room_a == target_2 || room_a == target_3)
    {
        final_class = room_a;
        target_room = 1;
        ROS_INFO("目标位于房间A,类型为%d", room_a);
    }
    if (room_b == target_1 || room_b == target_2 || room_b == target_3)
    {
        final_class = room_b;
        target_room = 2;
        ROS_INFO("目标位于房间B,类型为%d", room_b);
    }
    if (room_c == target_1 || room_c == target_2 || room_c == target_3)
    {
        final_class = room_c;
        target_room = 3;
        ROS_INFO("目标位于房间B,类型为%d", room_c);
    }
    
    ROS_INFO("开始发送数据");
    


    //此处开始向小车发送探测结果
    try {
        // 配置连接参数
        std::string host = "192.168.43.39";
        std::string port = "9090";
        std::string target = "/";
        
        // 初始化网络组件
        boost::asio::io_context ioc;
        boost::asio::ip::tcp::resolver resolver(ioc);
        websocket::stream<boost::asio::ip::tcp::socket> ws(ioc);

        // 建立TCP连接
        auto const results = resolver.resolve(host, port);
        boost::asio::connect(ws.next_layer(), results.begin(), results.end());
        
        // 完成WebSocket握手
        std::string host_with_port = host + ":" + port;
        ws.handshake(host_with_port, target);

        // 持续发送消息
        ROS_INFO("WebSocket客户端已启动,开始发布消息...");
        while (ros::ok()) {
            // 构造发布消息
            json publish_msg = {
                {"op", "publish"},
                {"topic", "/send_room_class"},
                {"msg", {
                    {"room", target_room},//
                    {"detected_class", final_class}//
                }}
            };

            // 发送消息
            ws.write(boost::asio::buffer(publish_msg.dump()));
            
            
            // 控制发布频率（1Hz）
            ros::Duration(1.0).sleep();
        }

    } catch (const std::exception& e) {
        ROS_ERROR("运行时错误: %s", e.what());
        return 1;
    }


    return 0;
}
 