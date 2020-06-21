#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <functional>

#include <boost/thread.hpp>

using namespace std::chrono_literals;
typedef websocketpp::server<websocketpp::config::asio> server;

#define NSEC_PER_SEC 1000000000
static inline uint64_t timespec_to_uint64(const struct timespec * t)
{
  return static_cast<uint64_t>(t->tv_sec) * NSEC_PER_SEC + static_cast<uint64_t>(t->tv_nsec);
}

class RclwsBridge : public rclcpp::Node
{
public:
    explicit RclwsBridge(const std::string & sub_topic_name, uint16_t port)
    : Node("rclcpp_websocket")
    {
        RCLCPP_INFO(this->get_logger(), "subscribe topic : %s\nserver port: %d", sub_topic_name.c_str(), port);
        // Set logging settings (/usr/local/includewebsocketpp/logger/levels.hpp)
        m_endpoint.set_error_channels(websocketpp::log::elevel::warn);
        m_endpoint.set_access_channels(websocketpp::log::alevel::all ^ websocketpp::log::alevel::frame_payload);

        // Initialize Asio
        m_endpoint.init_asio();

        // Set websocket handlers
        m_endpoint.set_open_handler(websocketpp::lib::bind(&RclwsBridge::on_open,this,std::placeholders::_1));
        m_endpoint.set_close_handler(websocketpp::lib::bind(&RclwsBridge::on_close,this,std::placeholders::_1));

        // Set the default message handler to the echo handler
        m_endpoint.set_message_handler(std::bind(
            &RclwsBridge::echo_handler, this,
            std::placeholders::_1, std::placeholders::_2
        ));

        // Create a callback function to subscribe
        auto callback =
            [this](const std_msgs::msg::String::UniquePtr msg) -> void
            {
                timespec curtime;
                clock_gettime(CLOCK_REALTIME, &curtime);
                std::string sendmsg = std::to_string(timespec_to_uint64(&curtime)) + " : "+ msg->data;
                RCLCPP_INFO(this->get_logger(), "%s",sendmsg.c_str());
                // Broadcast count to all connections
                con_list::iterator it;
                for (it = m_connections.begin(); it != m_connections.end(); ++it) {
                    m_endpoint.send(*it,sendmsg.c_str(),websocketpp::frame::opcode::text);
                }
            };

        // Topic setting for subscribe
        rclcpp::QoS qos(rclcpp::KeepLast(10));

        // Create subscriber
        sub_ = create_subscription<std_msgs::msg::String>(
          sub_topic_name, qos, callback);

        // Websocket server run
        ws_run(port);
    }

    void echo_handler(websocketpp::connection_hdl hdl, server::message_ptr msg) {
        // Write a new message
        m_endpoint.send(hdl, msg->get_payload(), msg->get_opcode());
    }

    void ws_run(uint16_t port) {
        // Listen on port 9002
        m_endpoint.listen(port);

        // Queues a connection accept operation
        m_endpoint.start_accept();

        // Start the ASIO io_service run loop
        boost::thread run_thread(boost::bind(&server::run, boost::ref(m_endpoint)));
    }

    void on_open(websocketpp::connection_hdl hdl) {
        m_connections.insert(hdl);
    }

    void on_close(websocketpp::connection_hdl hdl) {
        m_connections.erase(hdl);
    }

private:
    server m_endpoint;
    typedef std::set<websocketpp::connection_hdl,std::owner_less<websocketpp::connection_hdl>> con_list;
    con_list m_connections;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RclwsBridge>("chatter", 9002);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
