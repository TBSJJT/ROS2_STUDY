cmake_minimum_required(VERSION 3.8)
project(gasrobot_interfaces)

find_package(ament_cmake REQUIRED)

# Project-owned messages, services, and actions belong in this package.
# Add rosidl generators when the first stable interface is defined.
ament_package()
