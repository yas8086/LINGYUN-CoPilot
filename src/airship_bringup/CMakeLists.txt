cmake_minimum_required(VERSION 3.8)
project(airship_bringup)

find_package(ament_cmake REQUIRED)

# 安装launch启动文件目录
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装config参数配置目录
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 闭源代码: ament_copyright 无 proprietary license 模板,跳过版权检查
  # (若改用开源 license 如 Apache-2.0/BSD-3,删除下面一行即可启用版权检查)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
