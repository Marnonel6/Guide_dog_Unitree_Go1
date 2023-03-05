/// \file
/// \brief The nusim node is a simulation and visualization tool for the turtlebot3 robots.
///        It uses rviz2 for visualization and provides a simulated environment. The package
///        creates stationary walls and obstacles and track the position of a red robot.
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


class vision_obstacles : public rclcpp::Node
{
    public:
      vision_obstacles()
      : Node("vision_obstacles"), timestep_(0)
      {
        // Parameter descirption
        auto rate_des = rcl_interfaces::msg::ParameterDescriptor{};

        rate_des.description = "Timer callback frequency [Hz]";

        // Declare default parameters values
        declare_parameter("rate", 200, rate_des);     // Hz for timer_callback

        // Get params - Read params from yaml file that is passed in the launch file
        int rate = get_parameter("rate").get_parameter_value().get<int>();

        // Timer timestep [seconds]
        dt_ = 1.0 / static_cast<double>(rate);

        // // Create obstacles
        // create_obstacles_array();

        // Publishers
        timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
        obstacles_publisher_ =
          create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);

        //Subscribers
        door_vision_subscriber_ = create_subscription<geometry_msgs::msg::Point>(
          "/door", 10, std::bind(
            &vision_obstacles::door_vision_callback, this,
            std::placeholders::_1));


        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Timer
        timer_ = create_wall_timer(
          std::chrono::milliseconds(1000 / rate),
          std::bind(&vision_obstacles::timer_callback, this));
      }

    private:
      // Variables
      size_t timestep_;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
      double x_ = 0.0;
      double y_ = 0.0;
      double z_ = 0.0;
      double dt_ = 0.0; // Nusim Timer
      visualization_msgs::msg::MarkerArray obstacles_;

      // Create objects
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr timestep_publisher_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_publisher_;
      rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr door_vision_subscriber_;

        /// \brief Subscription callback function for /door object detection topic
        void door_vision_callback(const geometry_msgs::msg::Point & msg)
        {
            x_ = msg.x;
            // y_ = msg.y;
            // z_ = msg.z;
            z_ = msg.y;
            y_ = msg.z;
            RCLCPP_INFO_STREAM(get_logger(), "Coords" << x_ << y_ << z_);

            create_obstacles_array();
        }


      /// \brief (Just for testing) Broadcast the TF frames of the robot
      void broadcast_red_turtle()
      {
        geometry_msgs::msg::TransformStamped t_;

        t_.header.stamp = get_clock()->now();
        // t_.header.stamp.nanosec += 50000000; // TODO - Fixes the fake obstacle in Rviz
        t_.header.frame_id = "map";
        t_.child_frame_id = "base_link";
        t_.transform.translation.x = 1.0;
        t_.transform.translation.y = 1.0;
        t_.transform.translation.z = 0.0;     // Turtle only exists in 2D

        // Send the transformation
        tf_broadcaster_->sendTransform(t_);
      }

      /// \brief Create obstacles as a MarkerArray and publish them to a topic to display them in Rviz
      void create_obstacles_array()
      {
        if (x_ != 0.0 || y_ != 0.0 || z_ != 0.0)
        {
            obstacles_ = visualization_msgs::msg::MarkerArray{};
            visualization_msgs::msg::Marker obstacle_;
            obstacle_.header.frame_id = "base_link";
            obstacle_.header.stamp = get_clock()->now();
            obstacle_.id = 0;
            obstacle_.type = visualization_msgs::msg::Marker::CYLINDER;
            obstacle_.action = visualization_msgs::msg::Marker::ADD;
            obstacle_.pose.position.x = x_;
            obstacle_.pose.position.y = y_;
            obstacle_.pose.position.z = z_;
            obstacle_.pose.orientation.x = 0.0;
            obstacle_.pose.orientation.y = 0.0;
            obstacle_.pose.orientation.z = 0.0;
            obstacle_.pose.orientation.w = 1.0;
            obstacle_.scale.x = 0.5;   // Diameter in x
            obstacle_.scale.y = 0.5;   // Diameter in y
            obstacle_.scale.z = 0.5;         // Height
            obstacle_.color.r = 1.0f;
            obstacle_.color.g = 0.0f;
            obstacle_.color.b = 0.0f;
            obstacle_.color.a = 1.0;
            obstacles_.markers.push_back(obstacle_);

            obstacles_publisher_->publish(obstacles_);
        }

      }

      /// \brief Main simulation timer loop
      void timer_callback()
      {
        broadcast_red_turtle();
        // create_obstacles_array();

        auto message = std_msgs::msg::UInt64();
        message.data = timestep_++;

        timestep_publisher_->publish(message);
      }

      /// \brief Calculate the euclidean distance
      /// \param x1 point 1 x-coordinate (double)
      /// \param y1 point 1 y-coordinate (double)
      /// \param x2 point 2 x-coordinate (double)
      /// \param y2 point 2 y-coordinate (double)
      /// \return euclidean distance (double)
      double euclidean_distance(double x1, double y1, double x2, double y2)
      {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
      }
};

/// \brief Main function for node create, error handel and shutdown
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vision_obstacles>());
  rclcpp::shutdown();
  return 0;
}
