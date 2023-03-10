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
#include "geometry_msgs/msg/twist.hpp"


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
        // Publisher
        cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
          "cmd_vel", 10);

        // Subscribers
        voice_command_subscriber_ = create_subscription<std_msgs::msg::String>(
          "voice_command", 10, std::bind(
            &voice_control::voice_command_callback,
            this, std::placeholders::_1));

        //Services
        stand_up_client_ = create_client<std_srvs::srv::Empty>("recover_stand");
        lay_down_client_ = create_client<std_srvs::srv::Empty>("lay_down");
    }

private:
    // Variables
    bool service_done_stand_ = false; // inspired from action client c++ code
    bool service_done_lay_ = false; // inspired from action client c++ code
    geometry_msgs::msg::Twist body_twist_;
    // auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

    // Create objects
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_command_subscriber_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stand_up_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr lay_down_client_;


    /// \brief Voice command topic callback
    void voice_command_callback(const std_msgs::msg::String & msg)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "\n Command \n" << msg.data);

        if (msg.data == "stand" || msg.data == "up")
        {
            auto result_future_stand = stand_up_client_->async_send_request(
                std::make_shared<std_srvs::srv::Empty::Request>(), std::bind(&voice_control::response_callback_stand, this,
                                   std::placeholders::_1));
        }
        else if (msg.data == "lay")
        {
            auto result_future_lay = lay_down_client_->async_send_request(
                std::make_shared<std_srvs::srv::Empty::Request>(), std::bind(&voice_control::response_callback_lay, this,
                                   std::placeholders::_1));
        }
        else if (msg.data == "walk")
        {
            // TODO ADD GO TO GOAL SERVICE CALL
            body_twist_.linear.x = 0.4;
            body_twist_.angular.z = 0.0;
            cmd_vel_publisher_->publish(body_twist_);
        }
        else if (msg.data == "stop")
        {
            // TODO ADD CANCEL GOAL SERVICE CALL!!!
            body_twist_.linear.x = 0.0;
            body_twist_.angular.z = 0.0;
            cmd_vel_publisher_->publish(body_twist_);
        }
        else if (msg.data == "pet")
        {
            // TODO ADD RPY SERVICE call!!
        }
        else if (msg.data == "tail")
        {
            // TODO ADD RPY SERVICE call to wagg tail
        }
    }


    void response_callback_stand(
        rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready) {
                // uncomment below line if using Empty() message
                RCLCPP_ERROR_STREAM(get_logger(), "Result: success");
                // comment below line if using Empty() message
                //  RCLCPP_INFO(this->get_logger(), "Result: success: %i, message: %s",
                //              future.get()->success, future.get()->message.c_str());
                 service_done_stand_ = true;
            } 
            else {
                  RCLCPP_ERROR_STREAM(get_logger(), "Service In-Progress...");
            }
     }

    void response_callback_lay(
        rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready) {
                // uncomment below line if using Empty() message
                RCLCPP_ERROR_STREAM(get_logger(), "Result: success");
                // comment below line if using Empty() message
                //  RCLCPP_INFO(this->get_logger(), "Result: success: %i, message: %s",
                //              future.get()->success, future.get()->message.c_str());
                 service_done_lay_ = true;
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
