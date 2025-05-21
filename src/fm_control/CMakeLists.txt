cmake_minimum_required(VERSION 3.5)
project(turtlebot_formation)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()

