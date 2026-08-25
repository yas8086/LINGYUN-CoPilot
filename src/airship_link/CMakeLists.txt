cmake_minimum_required(VERSION 3.8)
project(airship_link)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(airship_msgs REQUIRED)

# ===== JSON 打包 + 串口接口 + UDP 发送 库 =====
add_library(${PROJECT_NAME}
  src/json_packer.cpp
  src/serial_interface.cpp
  src/udp_sender.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME} airship_msgs)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)

# ===== 串口数传链路节点 =====
add_executable(link_node src/link_node.cpp)
target_compile_features(link_node PUBLIC cxx_std_17)
target_link_libraries(link_node ${PROJECT_NAME})
ament_target_dependencies(link_node
  rclcpp std_msgs airship_msgs)

install(TARGETS link_node
  DESTINATION lib/${PROJECT_NAME})

# ===== 单元测试 =====
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 闭源代码: 跳过 ament_copyright 版权检查
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_json_packer
    test/test_json_packer.cpp
  )
  if(TARGET test_json_packer)
    target_link_libraries(test_json_packer ${PROJECT_NAME})
    ament_target_dependencies(test_json_packer airship_msgs)
  endif()

  ament_add_gtest(test_udp_sender
    test/test_udp_sender.cpp
  )
  if(TARGET test_udp_sender)
    target_link_libraries(test_udp_sender ${PROJECT_NAME})
  endif()
endif()

ament_package()