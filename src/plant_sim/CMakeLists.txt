cmake_minimum_required(VERSION 3.8)
project(plant_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pat_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(plant_sim_node src/plant_sim_node.cpp)
target_include_directories(plant_sim_node PRIVATE include)
ament_target_dependencies(plant_sim_node rclcpp pat_interfaces sensor_msgs std_msgs)

install(TARGETS
  plant_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_actuator test/test_actuator.cpp)
  target_include_directories(test_actuator PRIVATE include)
  ament_add_gtest(test_disturbance test/test_disturbance.cpp)
  target_include_directories(test_disturbance PRIVATE include)
endif()

ament_package()
