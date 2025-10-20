#include "px4_launch/px4_client.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto px4_client_node = std::make_shared<Px4Client>();

    px4_client_node->send_command_long();

    rclcpp::shutdown();
    return 0;
}