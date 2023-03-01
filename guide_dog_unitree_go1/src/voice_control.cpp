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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"



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
    }

private:
    // Variables

    // Create objects
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_command_subscriber_;


    /// \brief Voice command topic callback
    void voice_command_callback(const std_msgs::msg::String & msg)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "\n Command \n" << msg.data);
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
