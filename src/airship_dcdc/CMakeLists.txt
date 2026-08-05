cmake_minimum_required(VERSION 3.8)
project(airship_dcdc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(airship_msgs REQUIRED)
find_package(airship_can REQUIRED)
find_package(airship_utils REQUIRED)

# DCDC 驱动节点
add_executable(dcdc_node src/dcdc_node.cpp)
target_compile_features(dcdc_node PUBLIC cxx_std_17)
ament_target_dependencies(dcdc_node
  rclcpp std_msgs airship_msgs airship_can airship_utils)

install(TARGETS dcdc_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 闭源代码: 跳过 ament_copyright 版权检查
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()