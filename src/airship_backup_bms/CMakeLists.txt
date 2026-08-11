cmake_minimum_required(VERSION 3.8)
project(airship_backup_bms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(airship_msgs REQUIRED)
find_package(airship_link REQUIRED)

# ===== 可测试的北辰串口协议库(无 ROS 依赖) =====
add_library(${PROJECT_NAME}_proto src/backup_bms_protocol.cpp)
target_include_directories(${PROJECT_NAME}_proto PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_proto PUBLIC cxx_std_17)

# ===== 12S 备用电源 BMS 驱动节点 =====
add_executable(backup_bms_node src/backup_bms_node.cpp)
target_compile_features(backup_bms_node PUBLIC cxx_std_17)
ament_target_dependencies(backup_bms_node
  rclcpp builtin_interfaces airship_msgs airship_link)
target_link_libraries(backup_bms_node ${PROJECT_NAME}_proto)

install(TARGETS ${PROJECT_NAME}_proto
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)
install(TARGETS backup_bms_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 闭源代码: 跳过 ament_copyright 版权检查
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_backup_bms_protocol test/test_backup_bms_protocol.cpp)
  if(TARGET test_backup_bms_protocol)
    target_link_libraries(test_backup_bms_protocol ${PROJECT_NAME}_proto)
  endif()
endif()

ament_package()
