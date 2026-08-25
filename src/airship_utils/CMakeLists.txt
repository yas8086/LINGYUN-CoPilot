cmake_minimum_required(VERSION 3.8)
project(airship_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

# ===== 可测试的库(头文件 + 实现分离) =====
add_library(${PROJECT_NAME}
  src/math_utils.cpp
  src/can_utils.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/
  DESTINATION include)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)

# ===== gtest 单元测试 =====
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 闭源代码: 跳过 ament_copyright 版权检查
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_math_utils
    test/test_math_utils.cpp
  )
  if(TARGET test_math_utils)
    target_link_libraries(test_math_utils ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_can_utils
    test/test_can_utils.cpp
  )
  if(TARGET test_can_utils)
    target_link_libraries(test_can_utils ${PROJECT_NAME})
  endif()

  ament_add_gtest(test_offline_utils
    test/test_offline_utils.cpp
  )
  if(TARGET test_offline_utils)
    target_link_libraries(test_offline_utils ${PROJECT_NAME})
  endif()
endif()

ament_package()