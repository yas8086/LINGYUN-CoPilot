cmake_minimum_required(VERSION 3.8)
project(px4_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 本包仅含灵云定制消息(纯原生类型, 无 std_msgs/builtin_interfaces 依赖)
set(msg_files
  "msg/AirshipBladderPressure.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 闭源代码: ament_copyright 无 proprietary license 模板,跳过版权检查
  # (若改用开源 license 如 Apache-2.0/BSD-3,删除下面一行即可启用版权检查)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
