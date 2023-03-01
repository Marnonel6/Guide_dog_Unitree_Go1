/// \file
/// \brief
///
/// PARAMETERS:
///
/// PUBLISHES:
///
/// SUBSCRIBES:
///
/// SERVERS:
///
/// CLIENTS:
///
/// BROADCASTERS:

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <future>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/detail/empty__struct.hpp"
#include "std_srvs/srv/detail/set_bool__struct.hpp"
#include "std_srvs/srv/set_bool.hpp"


using namespace std::chrono_literals;

/// \brief Description
///
///  \param Name (Type): <definition>


class voice_control : public rclcpp::Node
{
public:
    voice_control()
    : Node("voice_control")
    {
        // Subscribers
        voice_command_subscriber_ = create_subscription<std_msgs::msg::String>(
          "voice_command", 10, std::bind(
            &voice_control::voice_command_callback,
            this, std::placeholders::_1));

        //Services
        // stand_up_client_ = create_client<std_srvs::srv::Empty>(
        //   "stand_up",
        //   std::bind(&voice_control::stand_up_callback, this,
        //             std::placeholders::_1, std::placeholders::_2));

        stand_up_client_ = create_client<std_srvs::srv::Empty>("stand_up");

    }

private:
    // Variables
    bool service_done_ = false; // inspired from action client c++ code
    // auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

    // Create objects
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_command_subscriber_;
    // Create the client for the node
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stand_up_client_;


    /// \brief Voice command topic callback
    void voice_command_callback(const std_msgs::msg::String & msg)
    {
        if (msg.data == "stand")
        {
            RCLCPP_ERROR_STREAM(get_logger(), "\n Command \n" << msg.data);
            // client->async_send_request(request);

            auto result_future = stand_up_client_->async_send_request(
                std::make_shared<std_srvs::srv::Empty::Request>(), std::bind(&voice_control::response_callback, this,
                                   std::placeholders::_1));
        }
    }


    void response_callback(
        rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready) {
                // uncomment below line if using Empty() message
                RCLCPP_ERROR_STREAM(get_logger(), "Result: success");
                // comment below line if using Empty() message
                //  RCLCPP_INFO(this->get_logger(), "Result: success: %i, message: %s",
                //              future.get()->success, future.get()->message.c_str());
                 service_done_ = true;
            } 
            else {
                  RCLCPP_ERROR_STREAM(get_logger(), "Service In-Progress...");
            }
     }
};

/// \brief Main function for node create and shutdown
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<voice_control>());
  rclcpp::shutdown();
  return 0;
}
