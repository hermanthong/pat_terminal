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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_estimator test/test_estimator.cpp)
  target_include_directories(test_estimator PRIVATE include)
  ament_add_gtest(test_mode_logic test/test_mode_logic.cpp)
  target_include_directories(test_mode_logic PRIVATE include)
endif()

ament_package()
