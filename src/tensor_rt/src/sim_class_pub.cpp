#include <iostream>
#include <string>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>
#include "ros/ros.h"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
using json = nlohmann::json;

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "websocket_publisher_node");
    ros::NodeHandle nh; 
    
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
                    {"room", 3},
                    {"detected_class", 5}
                }}
            };

            // 发送消息
            ws.write(boost::asio::buffer(publish_msg.dump()));
            ROS_INFO("已发布消息 - room: 2, detected_class: 5");
            
            // 控制发布频率（1Hz）
            ros::Duration(1.0).sleep();
        }

    } catch (const std::exception& e) {
        ROS_ERROR("运行时错误: %s", e.what());
        return 1;
    }
    return 0;
}