cmake_minimum_required(VERSION 3.8)
project(airship_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(airship_msgs REQUIRED)

# 安全监控节点: 订阅飞艇状态, 监控安全约束, 自动触发保护
add_executable(safety_monitor_node src/safety_monitor.cpp)
target_compile_features(safety_monitor_node PUBLIC cxx_std_17)
ament_target_dependencies(safety_monitor_node
  rclcpp std_msgs airship_msgs)

install(TARGETS safety_monitor_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
