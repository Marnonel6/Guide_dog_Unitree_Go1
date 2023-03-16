/// \file
/// \brief Subscribes to topics published by YOLOv7_ROS2 and publishes marker arrays in Rviz where
///        these obstacles are located.
///
/// PARAMETERS:
///     None
///
/// PUBLISHES:
///     \param ~/doors (visualization_msgs::msg::MarkerArray): Publishes a marker array in Rviz where
///                                                            the object is relative to the camera
///     \param ~/people (visualization_msgs::msg::MarkerArray): Publishes a marker array in Rviz where
///                                                            the object is relative to the camera
///     \param ~/stairs (visualization_msgs::msg::MarkerArray): Publishes a marker array in Rviz where
///                                                            the object is relative to the camera
///
/// SUBSCRIBES:
///     \param door (geometry_msgs::msg::Point): X,Y,Z coordinates of object relative to the camera
///     \param person (geometry_msgs::msg::Point): X,Y,Z coordinates of object relative to the camera
///     \param stairs (geometry_msgs::msg::Point): X,Y,Z coordinates of object relative to the camera
///
/// SERVERS:
///     None
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


/// \brief Subscribes to topics published by YOLOv7_ROS2 and publishes marker arrays in Rviz where
///        these obstacles are located.

class vision_obstacles : public rclcpp::Node
{
    public:
      vision_obstacles()
      : Node("vision_obstacles")
      {
        // Publishers
        // timestep_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
        obstacles_door_publisher_ =
          create_publisher<visualization_msgs::msg::MarkerArray>("~/doors", 10);
        obstacles_person_publisher_ =
          create_publisher<visualization_msgs::msg::MarkerArray>("~/people", 10);
        obstacles_stairs_publisher_ =
          create_publisher<visualization_msgs::msg::MarkerArray>("~/stairs", 10);

        //Subscribers
        door_vision_subscriber_ = create_subscription<geometry_msgs::msg::Point>(
          "/door", 10, std::bind(
            &vision_obstacles::door_vision_callback, this,
            std::placeholders::_1));
        person_vision_subscriber_ = create_subscription<geometry_msgs::msg::Point>(
          "/person", 10, std::bind(
            &vision_obstacles::person_vision_callback, this,
            std::placeholders::_1));
        stairs_vision_subscriber_ = create_subscription<geometry_msgs::msg::Point>(
          "/stairs", 10, std::bind(
            &vision_obstacles::stairs_vision_callback, this,
            std::placeholders::_1));

      }

    private:
      // Variables
      double x_door_ = 0.0;
      double y_door_ = 0.0;
      double z_door_ = 0.0;
      int id_door_ = 0;
      double x_person_ = 0.0;
      double y_person_ = 0.0;
      double z_person_ = 0.0;
      int id_person_ = 0;
      double x_stairs_ = 0.0;
      double y_stairs_ = 0.0;
      double z_stairs_ = 0.0;
      int id_stairs_ = 0;
      visualization_msgs::msg::MarkerArray obstacles_door_;
      visualization_msgs::msg::MarkerArray obstacles_person_;
      visualization_msgs::msg::MarkerArray obstacles_stairs_;

      // Create objects
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_door_publisher_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_person_publisher_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_stairs_publisher_;
      rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr door_vision_subscriber_;
      rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr person_vision_subscriber_;
      rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr stairs_vision_subscriber_;

      /// \brief Subscription callback function for /door object detection topic
      void door_vision_callback(const geometry_msgs::msg::Point & msg)
        {
            x_door_ = msg.x;
            z_door_ = msg.y;
            y_door_ = msg.z;
            create_door_obstacles_array();
        }

      /// \brief Create obstacles as a MarkerArray and publish them to a topic to display them in Rviz
      void create_door_obstacles_array()
      {
            obstacles_door_ = visualization_msgs::msg::MarkerArray{};
            visualization_msgs::msg::Marker obstacle_;
            obstacle_.header.frame_id = "camera_link";
            obstacle_.header.stamp = get_clock()->now();
            obstacle_.id = id_door_++;
            obstacle_.type = visualization_msgs::msg::Marker::CUBE;
            obstacle_.action = visualization_msgs::msg::Marker::ADD;
            obstacle_.pose.position.x = x_door_;
            obstacle_.pose.position.y = y_door_;
            obstacle_.pose.position.z = z_door_;
            obstacle_.pose.orientation.x = 0.0;
            obstacle_.pose.orientation.y = 0.0;
            obstacle_.pose.orientation.z = 0.0;
            obstacle_.pose.orientation.w = 1.0;
            obstacle_.scale.x = 0.8;   // Diameter in x
            obstacle_.scale.y = 0.1;   // Diameter in y
            obstacle_.scale.z = 2.0;         // Height
            obstacle_.color.r = 1.0f;
            obstacle_.color.g = 0.0f;
            obstacle_.color.b = 0.0f;
            obstacle_.color.a = 1.0;
            obstacle_.lifetime.nanosec = 100000000; // 0.1sec
            obstacles_door_.markers.push_back(obstacle_);

            obstacles_door_publisher_->publish(obstacles_door_);
      }

      /// \brief Subscription callback function for /door object detection topic
      void person_vision_callback(const geometry_msgs::msg::Point & msg)
      {
          x_person_ = msg.x;
          z_person_ = msg.y;
          y_person_ = msg.z;
          create_person_obstacles_array();
      }

      /// \brief Create obstacles as a MarkerArray and publish them to a topic to display them in Rviz
      void create_person_obstacles_array()
      {
            obstacles_person_ = visualization_msgs::msg::MarkerArray{};
            visualization_msgs::msg::Marker obstacle_;
            obstacle_.header.frame_id = "camera_link";
            obstacle_.header.stamp = get_clock()->now();
            obstacle_.id = id_person_++;
            obstacle_.type = visualization_msgs::msg::Marker::CYLINDER;
            obstacle_.action = visualization_msgs::msg::Marker::ADD;
            obstacle_.pose.position.x = x_person_;
            obstacle_.pose.position.y = y_person_;
            obstacle_.pose.position.z = z_person_;
            obstacle_.pose.orientation.x = 0.0;
            obstacle_.pose.orientation.y = 0.0;
            obstacle_.pose.orientation.z = 0.0;
            obstacle_.pose.orientation.w = 1.0;
            obstacle_.scale.x = 0.4;   // Diameter in x
            obstacle_.scale.y = 0.4;   // Diameter in y
            obstacle_.scale.z = 1.8;         // Height
            obstacle_.color.r = 0.0f;
            obstacle_.color.g = 1.0f;
            obstacle_.color.b = 0.0f;
            obstacle_.color.a = 1.0;
            obstacle_.lifetime.nanosec = 100000000; // 0.1sec
            obstacles_person_.markers.push_back(obstacle_);

            obstacles_person_publisher_->publish(obstacles_person_);
      }

      /// \brief Subscription callback function for /door object detection topic
      void stairs_vision_callback(const geometry_msgs::msg::Point & msg)
      {
          x_stairs_ = msg.x;
          z_stairs_ = msg.y;
          y_stairs_ = msg.z;
          create_stairs_obstacles_array();
      }

      /// \brief Create obstacles as a MarkerArray and publish them to a topic to display them in Rviz
      void create_stairs_obstacles_array()
      {
            obstacles_stairs_ = visualization_msgs::msg::MarkerArray{};
            visualization_msgs::msg::Marker obstacle_;
            obstacle_.header.frame_id = "camera_link";
            obstacle_.header.stamp = get_clock()->now();
            obstacle_.id = id_stairs_++;
            obstacle_.type = visualization_msgs::msg::Marker::CUBE;
            obstacle_.action = visualization_msgs::msg::Marker::ADD;
            obstacle_.pose.position.x = x_stairs_;
            obstacle_.pose.position.y = y_stairs_;
            obstacle_.pose.position.z = z_stairs_;
            obstacle_.pose.orientation.x = 0.0;
            obstacle_.pose.orientation.y = 0.0;
            obstacle_.pose.orientation.z = 0.0;
            obstacle_.pose.orientation.w = 1.0;
            obstacle_.scale.x = 0.5;   // Diameter in x
            obstacle_.scale.y = 0.5;   // Diameter in y
            obstacle_.scale.z = 0.5;         // Height
            obstacle_.color.r = 0.0f;
            obstacle_.color.g = 0.0f;
            obstacle_.color.b = 1.0f;
            obstacle_.color.a = 1.0;
            obstacle_.lifetime.nanosec = 100000000; // 0.1sec
            obstacles_stairs_.markers.push_back(obstacle_);

            obstacles_stairs_publisher_->publish(obstacles_stairs_);
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
