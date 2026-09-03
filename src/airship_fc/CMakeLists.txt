cmake_minimum_required(VERSION 3.8)
project(airship_fc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(airship_msgs REQUIRED)
find_package(airship_utils REQUIRED)
find_package(px4_msgs REQUIRED)

# ===== 飞控监控节点 =====
add_executable(fc_monitor_node src/fc_monitor_node.cpp)
target_compile_features(fc_monitor_node PUBLIC cxx_std_17)
target_include_directories(fc_monitor_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(fc_monitor_node
  rclcpp std_msgs sensor_msgs nav_msgs mavros_msgs airship_msgs airship_utils)

install(TARGETS fc_monitor_node
  DESTINATION lib/${PROJECT_NAME})

# ===== 气囊压差桥节点 (LoRa -> uXRCE-DDS -> PX4) =====
add_executable(bladder_bridge_node src/bladder_bridge_node.cpp)
target_compile_features(bladder_bridge_node PUBLIC cxx_std_17)
target_include_directories(bladder_bridge_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(bladder_bridge_node
  rclcpp airship_msgs px4_msgs)

install(TARGETS bladder_bridge_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 闭源代码: 跳过 ament_copyright 版权检查
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # 告警规则引擎纯逻辑单测(header-only, 无 rclcpp 依赖)
  ament_add_gtest(test_fc_alert_logic
    test/test_fc_alert_logic.cpp
  )
  if(TARGET test_fc_alert_logic)
    # fc_alert_logic.hpp 仅依赖 <cstdint>, 无需链接业务库
    target_include_directories(test_fc_alert_logic PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  endif()

  # 气囊压差桥接核心逻辑单测(header-only, 无 rclcpp 依赖)
  ament_add_gtest(test_bladder_bridge_logic
    test/test_bladder_bridge_logic.cpp
  )
  if(TARGET test_bladder_bridge_logic)
    target_include_directories(test_bladder_bridge_logic PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  endif()
endif()

ament_package()