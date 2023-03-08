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
///     None
///
/// BROADCASTERS:

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;


/// \brief This class publishes the current timestep of the simulation, obstacles and walls that
///
///  \param rate (int): Timer callback frequency [Hz]


class guide_dog : public rclcpp::Node
{
    public:
      guide_dog()
      : Node("guide_dog"), timestep_(0)
      {
        // Parameter descirption
        auto rate_des = rcl_interfaces::msg::ParameterDescriptor{};
        rate_des.description = "Timer callback frequency [Hz]";

        // Declare default parameters values
        declare_parameter("rate", 200, rate_des);     // Hz for timer_callback

        // Get params - Read params from yaml file that is passed in the launch file
        int rate = get_parameter("rate").get_parameter_value().get<int>();

        // Publishers
        //Subscribers

        // Timer
        timer_ = create_wall_timer(
          std::chrono::milliseconds(1000 / rate),
          std::bind(&guide_dog::timer_callback, this));
      }

    private:
      // Variables
      size_t timestep_;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

      // Create objects
      rclcpp::TimerBase::SharedPtr timer_;

      /// \brief Main simulation timer loop
      void timer_callback()
      {

      }
};

/// \brief Main function for node create, error handel and shutdown
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<guide_dog>());
  rclcpp::shutdown();
  return 0;
}
