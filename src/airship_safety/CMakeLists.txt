cmake_minimum_required(VERSION 3.8)
project(airship_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(airship_msgs REQUIRED)

# ===== 纯逻辑库(无 ROS 依赖, 便于单元测试) =====
add_library(${PROJECT_NAME}_logic
  src/safety_logic.cpp
)
target_include_directories(${PROJECT_NAME}_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_logic PUBLIC cxx_std_17)

# 安全仲裁节点
add_executable(safety_node src/safety_node.cpp)
target_compile_features(safety_node PUBLIC cxx_std_17)
ament_target_dependencies(safety_node
  rclcpp std_msgs airship_msgs)
target_link_libraries(safety_node ${PROJECT_NAME}_logic)

install(TARGETS safety_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_logic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # 闭源代码: 跳过 ament_copyright 版权检查
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_safety_logic test/test_safety_logic.cpp)
  target_link_libraries(test_safety_logic ${PROJECT_NAME}_logic)
endif()

ament_package()