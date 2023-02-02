cmake_minimum_required(VERSION 3.8)
project(dua_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/CommandResult.msg
  msg/CommandResultStamped.msg
  msg/CoordinateSystem.msg
  msg/EulerAttitudeSetpoint.msg
  msg/EulerPoseStamped.msg
  msg/PositionSetpoint.msg
  msg/VelocitySetpoint.msg
  DEPENDENCIES geometry_msgs std_msgs)

ament_package()
