cmake_minimum_required(VERSION 3.8)
project(pat_terminal)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pat_interfaces REQUIRED)

add_executable(mode_manager src/mode_manager.cpp)
target_include_directories(mode_manager PRIVATE include)
ament_target_dependencies(mode_manager rclcpp std_msgs pat_interfaces)

add_executable(fine_controller src/fine_controller.cpp)
target_include_directories(fine_controller PRIVATE include)
ament_target_dependencies(fine_controller rclcpp sensor_msgs pat_interfaces)

add_executable(coarse_controller src/coarse_controller.cpp)
target_include_directories(coarse_controller PRIVATE include)
ament_target_dependencies(coarse_controller rclcpp pat_interfaces)

install(TARGETS
  mode_manager
  fine_controller
  coarse_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_estimator test/test_estimator.cpp)
  target_include_directories(test_estimator PRIVATE include)
  ament_add_gtest(test_mode_logic test/test_mode_logic.cpp)
  target_include_directories(test_mode_logic PRIVATE include)
  ament_add_gtest(test_pi_controller test/test_pi_controller.cpp)
  target_include_directories(test_pi_controller PRIVATE include)
  ament_add_gtest(test_low_pass test/test_low_pass.cpp)
  target_include_directories(test_low_pass PRIVATE include)
endif()

ament_package()
