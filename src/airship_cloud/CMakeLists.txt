cmake_minimum_required(VERSION 3.8)
project(airship_cloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(airship_msgs REQUIRED)
find_package(airship_link REQUIRED)

# libmosquitto (C 库, 无 CMake config, 手动查找)
find_library(MOSQUITTO_LIBRARY REQUIRED NAMES mosquitto)

# ===== MQTT 客户端库 =====
add_library(${PROJECT_NAME}_mqtt src/mqtt_client.cpp)
target_include_directories(${PROJECT_NAME}_mqtt PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_mqtt PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_mqtt PUBLIC ${MOSQUITTO_LIBRARY})

# ===== 上云节点 =====
add_executable(cloud_node src/cloud_node.cpp)
target_compile_features(cloud_node PUBLIC cxx_std_17)
ament_target_dependencies(cloud_node
  rclcpp airship_msgs airship_link)
target_link_libraries(cloud_node ${PROJECT_NAME}_mqtt)

install(TARGETS ${PROJECT_NAME}_mqtt
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)
install(TARGETS cloud_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 闭源代码: 跳过 ament_copyright 版权检查
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()