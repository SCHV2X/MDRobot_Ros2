cmake_minimum_required(VERSION 3.5)
project(md)

# C++ 표준 설정
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 의존성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 메시지 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MdRobotMsg1.msg"
  "msg/MdRobotMsg2.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

# rosidl_default_runtime 의존성 내보내기
ament_export_dependencies(rosidl_default_runtime)

# 헤더 파일 포함 경로
include_directories(
  include
)

# 실행 파일 생성
add_executable(md_robot_node
  src/md_robot_node/main.cpp
  src/md_robot_node/com.cpp
  src/md_robot_node/robot.cpp
)

# 실행 파일의 의존성 설정
ament_target_dependencies(md_robot_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  serial
)

# 실행 파일이 메시지 생성 후 빌드되도록 설정
add_dependencies(md_robot_node ${PROJECT_NAME}_generate_interfaces_cpp)

# 실행 파일 설치
install(TARGETS md_robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# include 디렉토리 설치
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
