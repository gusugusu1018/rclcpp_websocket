#include "rclcpp_websocket/rclcpp_websocket.hpp"

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    rclcpp::init(argc, argv);
    std::uint16_t websocketPort = 9002;
    auto node = std::make_shared<rclcpp_websocket::RclcppWebsocket>("chatter", websocketPort);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
