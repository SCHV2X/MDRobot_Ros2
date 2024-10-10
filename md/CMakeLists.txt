cmake_minimum_required(VERSION 3.5)
project(md)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial REQUIRED)

# Generate messages in the 'msg' folder
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MdRobotMsg1.msg"
  "msg/MdRobotMsg2.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# Include directories
include_directories(
  include
)

# Declare a C++ executable
add_executable(md_robot_node src/md_robot_node/main.cpp src/md_robot_node/com.cpp src/md_robot_node/robot.cpp)
ament_target_dependencies(md_robot_node rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs tf2_ros tf2_geometry_msgs serial)

# Install executable
install(TARGETS md_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()