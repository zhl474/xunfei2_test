#include <iostream>
#include <string>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>
#include "ros/ros.h"

namespace beast = boost::beast;//简化nlohmann库中json的名称
namespace websocket = beast::websocket;//简化beast库中websocket的名称
using json = nlohmann::json;//将nlohmann的JSON库设为默认的JSON解析器

// 全局变量存储目标类别
uint16_t class_1 = 0;
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

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "websocket_client_node");
    
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
    return 0;
}