#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <functional>

using json = nlohmann::json;
using namespace std::placeholders;
typedef websocketpp::server<websocketpp::config::asio> server;

class CarServer {
public:
    CarServer(int port) : m_port(port), m_request_count(0) {
        m_server.init_asio();
        m_server.set_open_handler(std::bind(&CarServer::on_open, this, _1));
        m_server.set_message_handler(std::bind(&CarServer::on_mes
    }

    void on_open(websocketpp::connection_hdl hdl) {
        std::cout << "客户端连接成功" << std::endl;
    }

    void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg) {
        json req = json::parse(msg->get_payload());
        
        json res;
        if(req["op"] == "call_service" && req["service"] == "/car_service") {
            int room = req["args"]["room"];
            int cls = req["args"]["class"];
            
            if(room == -1 && cls == -1) { // 首次请求处理
                res = {
                    {"op", "service_response"},
                    {"service", "/car_service"},
                    {"values", {{"target_class", 2}}}
                };
                m_request_count = 1;
            } else if(m_request_count == 1) { // 第二次请求处理
                m_target_room = room;
                m_class_2 = cls;
                res = {
                    {"op", "service_response"},
                    {"service", "/car_service"},
                    {"values", {{"target_class", 0}}}
                };
                m_server.close(hdl, websocketpp::close::status::normal, ""); // 关闭连接
            }
            m_server.send(hdl, res.dump(), websocketpp::frame::opcode::text);
        }
    }

    void run() {
        m_server.listen(m_port);
        m_server.start_accept();
        m_server.run();
    }

private:
    server m_server;
    int m_port;
    int m_target_room;
    int m_class_2;
    int m_request_count;
};

int main() {
    CarServer car_server(9090);
    std::cout << "实体车服务端启动..." << std::endl;
    car_server.run();
    return 0;
}