cmake_minimum_required(VERSION 3.8)
project(pat_terminal)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pat_interfaces REQUIRED)

add_executable(mode_manager src/mode_manager.cpp)
ament_target_dependencies(mode_manager rclcpp std_msgs pat_interfaces)

install(TARGETS
  mode_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
