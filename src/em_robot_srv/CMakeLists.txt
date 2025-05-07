cmake_minimum_required(VERSION 3.5)
project(em_robot_srv)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetInitialPose.srv"
)

# Ensure that message generation happens before Python setup
ament_export_dependencies(rosidl_default_runtime)
ament_package()
