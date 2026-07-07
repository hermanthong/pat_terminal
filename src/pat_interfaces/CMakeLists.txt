cmake_minimum_required(VERSION 3.8)
project(pat_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ModeState.msg"
  "msg/PositionError.msg"
  "msg/AxisState.msg"
  "msg/AxisCommand.msg"
  "srv/SetMode.srv"
  "srv/SetBearing.srv"
  DEPENDENCIES std_msgs
)

ament_package()
