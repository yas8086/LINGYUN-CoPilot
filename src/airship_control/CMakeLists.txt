cmake_minimum_required(VERSION 3.8)
project(airship_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(airship_msgs REQUIRED)

# Offboard控制器节点
add_executable(offboard_controller src/offboard_controller.cpp)
target_include_directories(offboard_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(offboard_controller PUBLIC cxx_std_17)
ament_target_dependencies(offboard_controller
  rclcpp std_msgs geometry_msgs airship_msgs)

install(TARGETS offboard_controller
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
