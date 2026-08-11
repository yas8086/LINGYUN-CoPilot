cmake_minimum_required(VERSION 3.8)
project(airship_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/BmsStatus.msg"
  "msg/BackupBmsStatus.msg"
  "msg/MpptStatus.msg"
  "msg/DcdcStatus.msg"
  "msg/DeviceAlert.msg"
  "msg/FlightStatus.msg"
  "msg/FcAlert.msg"
  "msg/LoRaSample.msg"
  "msg/LoRaSamples.msg"
  "msg/LoRaSummary.msg"
  "msg/SafetyStatus.msg"
)

set(srv_files
  "srv/QueryNode.srv"
  "srv/QuerySummary.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs builtin_interfaces
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
