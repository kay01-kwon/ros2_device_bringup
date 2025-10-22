#include "px4_client.hpp"

Px4Client::Px4Client()
: Node("px4_client_node")
{
    std::string service_name;
    service_name = "/mavros/cmd/command";

    client_ = this->create_client<mavros_msgs::srv::CommandLong>(service_name);

  RCLCPP_INFO(get_logger(), "Waiting for service '%s' ...", service_name.c_str());
  while (!client_->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for '%s'", service_name.c_str());
      return;
    }
    RCLCPP_WARN(get_logger(), "Still waiting for '%s' ...", service_name.c_str());
  }
  RCLCPP_INFO(get_logger(), "Service '%s' is available.", service_name.c_str());

}

Px4Client::~Px4Client()
{
}

void Px4Client::send_command_long()
{
    std::string msg_name;

    auto request = std::make_shared<CommandLong::Request>();

    request->command = MAV_CMD_SET_MESSAGE_INTERVAL_;
    request->confirmation = 0;
    request->broadcast = false;

    RCLCPP_INFO(
        this->get_logger(), 
        "Waiting for service to be available...");

    while(rclcpp::ok())
    {
        RCLCPP_INFO(
        this->get_logger(), 
        "Setting message rates...");
        
        // 1. Set HIGHRES_IMU rate
        request->param1 = HIGHRES_IMU_ID_;
        request->param2 = HIGHRES_IMU_RATE_;
        msg_name = "HIGHRES_IMU";
        call_service(request, is_set_HIGHRES_IMU_RATE_, msg_name);
        RCLCPP_INFO(
            this->get_logger(), 
            "HIGHRES_IMU rate set to: %.2f microseconds", HIGHRES_IMU_RATE_);


        // 2. Set ATTITUDE_QUATERNION rate
        request->param1 = ATTITUDE_QUATERNION_ID_;
        request->param2 = ATTITUDE_QUATERNION_RATE_;
        msg_name = "ATTITUDE_QUATERNION";
        call_service(request, is_set_ATTITUDE_QUATERNION_RATE_, msg_name);
        RCLCPP_INFO(
        this->get_logger(), 
        "ATTITUDE_QUATERNION rate set to: %.2f microseconds", ATTITUDE_QUATERNION_RATE_);

        // 3. Set RC_CHANNELS rate
        request->param1 = RC_CHANNELS_ID_;
        request->param2 = RC_CHANNELS_RATE_;
        msg_name = "RC_CHANNELS";
        call_service(request, is_set_RC_CHANNELS_RATE_, msg_name);
        RCLCPP_INFO(
        this->get_logger(),
        "RC_CHANNELS rate set to: %.2f microseconds", RC_CHANNELS_RATE_);

        // 4. Set LOCAL_POSITION_ODOM rate
        request->param1 = LOCAL_POSITION_ODOM_ID_;
        request->param2 = LOCAL_POSITION_ODOM_RATE_;
        msg_name = "LOCAL_POSITION_ODOM";
        call_service(request, is_set_LOCAL_POSITION_ODOM_RATE_, msg_name);
        RCLCPP_INFO(
        this->get_logger(),
        "LOCAL_POSITION_ODOM rate set to: %.2f microseconds", LOCAL_POSITION_ODOM_RATE_);

        if(is_set_HIGHRES_IMU_RATE_ && is_set_ATTITUDE_QUATERNION_RATE_ &&
           is_set_RC_CHANNELS_RATE_ && is_set_LOCAL_POSITION_ODOM_RATE_)
        {
            RCLCPP_INFO(
                this->get_logger(), "All message rates have been set successfully.");
            break;
        }
    }
}

void Px4Client::call_service(std::shared_ptr<CommandLong::Request> req, 
    bool &is_set_rate, const std::string &msg_name)
{
    if(!is_set_rate)
    {
        auto result_future = client_->async_send_request(req);

        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(
                this->get_logger(), "Successfully set %s rate.", msg_name.c_str());
            is_set_rate = true;
        }
        else
        {
            RCLCPP_ERROR(
                this->get_logger(), "Failed to call service to set %s rate.", msg_name.c_str());
        }
    }
}