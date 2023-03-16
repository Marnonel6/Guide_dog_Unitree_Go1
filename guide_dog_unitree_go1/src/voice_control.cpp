/// \file
/// \brief Voice control node that subscribes to the voice command topic and preforms an action
///        depending on the topic.
///
/// PARAMETERS:
///
/// PUBLISHES:
///     \param cmd_vel (geometry_msgs::msg::Twist): Publishes to /cmd_vel to move the robot
///
/// SUBSCRIBES:
///     \param voice_command (std_msgs::msg::String): Subscribes to the topic and preforms an action
///                                                   depending on the topic
///
/// SERVERS:
///
/// CLIENTS:
///     \param recover_stand (std_srvs::srv::Empty): Sets Go1 in a stable recovered stand pose
///     \param lay_down (std_srvs::srv::Empty): Makes Go1 lay down
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

/// \brief Voice control node that subscribes to the voice command topic and preforms an action
///        depending on the topic.
///
///  \param body_twist_ (geometry_msgs::msg::Twist): Desired body twist
///  \param service_done_lay_ (bool): Flag for service client
///  \param service_done_stand_ (bool): Flag for service client


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
    bool service_done_stand_ = false;
    bool service_done_lay_ = false;
    geometry_msgs::msg::Twist body_twist_;

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
    }


    void response_callback_stand(
        rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
            auto status = future.wait_for(1s);
            if (status == std::future_status::ready) {
                RCLCPP_ERROR_STREAM(get_logger(), "Result: success");
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
                RCLCPP_ERROR_STREAM(get_logger(), "Result: success");
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
