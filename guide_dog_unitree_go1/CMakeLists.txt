cmake_minimum_required(VERSION 3.8)
project(guide_dog_unitree_go1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(unitree_nav_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(ros2_unitree_legged_msgs REQUIRED)

if(NOT CMAKE_CROSSCOMPILING)
    # CMake also has the ability to generate doxygen documentation
    find_package(Doxygen)
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md) # Use the readme in your doxygen docs
    doxygen_add_docs(doxygen src/ README.md ALL)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(voice_control src/voice_control.cpp)
ament_target_dependencies(voice_control rclcpp std_msgs std_srvs tf2 tf2_ros nav_msgs sensor_msgs geometry_msgs)
add_executable(vision_obstacles src/vision_obstacles.cpp)
ament_target_dependencies(vision_obstacles rclcpp std_msgs std_srvs tf2 tf2_ros visualization_msgs nav_msgs sensor_msgs)
add_executable(guide_dog src/guide_dog.cpp)
ament_target_dependencies(guide_dog rclcpp rclcpp_action geometry_msgs std_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs nav2_msgs visualization_msgs nav_msgs sensor_msgs unitree_nav_interfaces)


install(TARGETS
  voice_control
  vision_obstacles
  guide_dog
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
