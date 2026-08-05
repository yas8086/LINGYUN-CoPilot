cmake_minimum_required(VERSION 3.8)
project(airship_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(airship_msgs REQUIRED)

# 监控聚合节点
add_executable(monitor_node src/monitor_node.cpp)
target_compile_features(monitor_node PUBLIC cxx_std_17)
ament_target_dependencies(monitor_node
  rclcpp std_msgs airship_msgs)

install(TARGETS monitor_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 闭源代码: 跳过 ament_copyright 版权检查
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()