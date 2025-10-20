#ifndef PX4_CLIENT_HPP
#define PX4_CLIENT_HPP


#include <chrono>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_long.hpp>

using mavros_msgs::srv::CommandLong;
using namespace std::chrono_literals;

class Px4Client : public rclcpp::Node
{
    public:

    Px4Client();

    void send_command_long();

    ~Px4Client();

    private:

    void call_service(std::shared_ptr<CommandLong::Request> req, 
    bool &is_set_rate, 
    const std::string &msg_name);
    
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr client_;

    bool is_set_HIGHRES_IMU_RATE_{false};
    bool is_set_ATTITUDE_QUATERNION_RATE_{false};
    bool is_set_RC_CHANNELS_RATE_{false};
    bool is_set_LOCAL_POSITION_ODOM_RATE_{false};

    // Message IDs
    float MAV_CMD_SET_MESSAGE_INTERVAL_{511.0f};
    float HIGHRES_IMU_ID_{105.0f};
    float ATTITUDE_QUATERNION_ID_{31.0f};
    float RC_CHANNELS_ID_{65.0f};
    float LOCAL_POSITION_ODOM_ID_{32.0f};

    // 200 Hz default rates
    float HIGHRES_IMU_RATE_{5000.0f};
    float ATTITUDE_QUATERNION_RATE_{5000.0f};
    // 100 Hz default rates
    float RC_CHANNELS_RATE_{10000.0f};
    float LOCAL_POSITION_ODOM_RATE_{10000.0f};

};


#endif // PX4_CLIENT_HPP