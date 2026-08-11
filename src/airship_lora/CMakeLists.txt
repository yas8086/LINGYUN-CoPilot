cmake_minimum_required(VERSION 3.8)
project(airship_lora)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(airship_msgs REQUIRED)
find_package(airship_link REQUIRED)

# ===== 可测试的 Modbus RTU 协议库(无 ROS 依赖) =====
add_library(${PROJECT_NAME}_rtu src/modbus_rtu.cpp)
target_include_directories(${PROJECT_NAME}_rtu PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_rtu PUBLIC cxx_std_17)

# ===== LoRa 采集节点 =====
add_executable(lora_node src/lora_node.cpp)
target_compile_features(lora_node PUBLIC cxx_std_17)
ament_target_dependencies(lora_node
  rclcpp builtin_interfaces airship_msgs airship_link)
target_link_libraries(lora_node ${PROJECT_NAME}_rtu)

install(TARGETS ${PROJECT_NAME}_rtu
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)
install(TARGETS lora_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 闭源代码: 跳过 ament_copyright 版权检查
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_modbus_rtu test/test_modbus_rtu.cpp)
  if(TARGET test_modbus_rtu)
    target_link_libraries(test_modbus_rtu ${PROJECT_NAME}_rtu)
  endif()
endif()

ament_package()
