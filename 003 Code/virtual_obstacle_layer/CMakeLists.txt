cmake_minimum_required(VERSION 3.8)
project(virtual_obstacle_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# include 디렉토리
include_directories(
  include
)

# 라이브러리 생성
add_library(${PROJECT_NAME} SHARED
  src/virtual_obstacle_layer.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav2_costmap_2d
  nav_msgs
  pluginlib
)

# pluginlib export
pluginlib_export_plugin_description_file(nav2_costmap_2d plugin.xml)

# 설치 경로 지정
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp nav2_costmap_2d nav_msgs pluginlib)

ament_package()
