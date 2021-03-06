#ifndef RCLCPP_WEBSOCKET_HPP
#define RCLCPP_WEBSOCKET_HPP

#include <cstdio>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <functional>

#include <boost/thread.hpp>

namespace rclcpp_websocket
{

    using namespace std::chrono_literals;
    typedef websocketpp::server<websocketpp::config::asio> server;

    class RclcppWebsocket : public rclcpp::Node
    {
    public:
        explicit RclcppWebsocket(const std::string & sub_topic_name, std::uint16_t port)
        : Node("rclcpp_websocket")
        {
            RCLCPP_INFO(this->get_logger(), "subscribe topic : %s\nserver port: %d", sub_topic_name.c_str(), port);
            // Set logging settings (/usr/local/includewebsocketpp/logger/levels.hpp)
            endpoint_.set_error_channels(websocketpp::log::elevel::warn);
            endpoint_.set_access_channels(websocketpp::log::alevel::all ^ websocketpp::log::alevel::frame_payload);

            // Initialize Asio
            endpoint_.init_asio();

            // Set websocket handlers
            endpoint_.set_open_handler(websocketpp::lib::bind(&RclcppWebsocket::on_open,this,std::placeholders::_1));
            endpoint_.set_close_handler(websocketpp::lib::bind(&RclcppWebsocket::on_close,this,std::placeholders::_1));

            // Set the default message handler to the echo handler
            endpoint_.set_message_handler(std::bind(
                &RclcppWebsocket::echo_handler, this,
                std::placeholders::_1, std::placeholders::_2
            ));

            // Create a callback function to subscribe
            auto callback =
                [this](const std_msgs::msg::String::UniquePtr msg) -> void
                {
                    RCLCPP_INFO(this->get_logger(), "%s",msg->data.c_str());
                    // Broadcast count to all connections
                    con_list::iterator it;
                    for (it = connections_.begin(); it != connections_.end(); ++it) {
                        endpoint_.send(*it,msg->data.c_str(),websocketpp::frame::opcode::text);
                    }
                };

            // Topic setting for subscribe
            rclcpp::QoS qos(rclcpp::KeepLast(10));

            // Create subscriber
            sub_ = create_subscription<std_msgs::msg::String>(sub_topic_name, qos, callback);

            // Websocket server run
            RclcppWebsocket::ws_run(port);
        }

        void ws_run(std::uint16_t port){
            // Listen on port 9002
            endpoint_.listen(port);

            // Queues a connection accept operation
            endpoint_.start_accept();

            // Start the ASIO io_service run loop
            boost::thread run_thread(boost::bind(&server::run, boost::ref(endpoint_)));
        }

        void echo_handler(websocketpp::connection_hdl hdl, server::message_ptr msg) {
            // Write a new message
            endpoint_.send(hdl, msg->get_payload(), msg->get_opcode());
        }

        void on_open(websocketpp::connection_hdl hdl) {
            connections_.insert(hdl);
        }

        void on_close(websocketpp::connection_hdl hdl) {
            connections_.erase(hdl);
        }

    private:
        server endpoint_;
        typedef std::set<websocketpp::connection_hdl,std::owner_less<websocketpp::connection_hdl>> con_list;
        con_list connections_;

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    };

} // namespace rclcpp_websocket

#endif // RCLCPP_WEBSOCKET_HPP