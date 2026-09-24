cmake_minimum_required(VERSION 3.8)
project(unistackbot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unistackbot_interface REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 编排层最小版 (0d, 2026-09-21): 非RT诊断汇聚 → /supervisor/alerts
add_executable(supervisor_node src/supervisor_node.cpp)
target_compile_features(supervisor_node PUBLIC cxx_std_17)
target_include_directories(supervisor_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common>              # ulog
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
)
ament_target_dependencies(supervisor_node
  rclcpp diagnostic_msgs sensor_msgs geometry_msgs unistackbot_interface
)
install(TARGETS supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

# 控制 demo 已独立成包 unistackbot_demo (2026-09-23): 三命令域演示程序 (末端/关节慢/关节满速)
# scripts/ 目录随之退役 (空目录不保留)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
