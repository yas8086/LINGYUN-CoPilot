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
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
