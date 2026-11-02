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
find_package(unistackbot_sim_control REQUIRED)   # 操作态通道契约头 (kPausedTopic)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 编排层最小版 (0d, 2026-09-21): 非RT诊断汇聚 → /supervisor/alerts
# 通用总线 SystemInterface 插件 (2026-11-02 组合根裁决): 一条代码服务所有 MasterBase
# 总线——总线差异住配置(URDF)与总线包; registerKnownBuses 链接各总线工厂注册。
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(unistackbot_protocol REQUIRED)
find_package(unistackbot_statemachine REQUIRED)
find_package(unistackbot_bus REQUIRED)
find_package(unistackbot_udp_board REQUIRED)
add_library(unistackbot_bus_system SHARED src/bus_system.cpp)
target_compile_features(unistackbot_bus_system PUBLIC cxx_std_17)
target_include_directories(unistackbot_bus_system PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/src
	${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common
)
ament_target_dependencies(unistackbot_bus_system
	hardware_interface pluginlib rclcpp rclcpp_lifecycle
	unistackbot_protocol unistackbot_statemachine unistackbot_bus unistackbot_udp_board
)
# 工厂注册符号 (老式 ament 包无导出目标: 经 LIBRARIES/LIBRARY_DIRS 变量取路径)
target_link_libraries(unistackbot_bus_system ${unistackbot_udp_board_LIBRARIES})
target_link_directories(unistackbot_bus_system PRIVATE ${unistackbot_udp_board_LIBRARY_DIRS})
install(TARGETS unistackbot_bus_system
	ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
)
pluginlib_export_plugin_description_file(hardware_interface bus_system.xml)
install(FILES bus_system.xml DESTINATION share/${PROJECT_NAME})

add_executable(supervisor_node src/supervisor_node.cpp)
target_compile_features(supervisor_node PUBLIC cxx_std_17)
target_include_directories(supervisor_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common>              # ulog
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
)
ament_target_dependencies(supervisor_node
  rclcpp diagnostic_msgs sensor_msgs geometry_msgs unistackbot_interface
  unistackbot_sim_control
)
install(TARGETS supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

# 控制 demo 已独立成包 unistackbot_demo (2026-09-23): 三命令域演示程序 (末端/关节慢/关节满速)
# scripts/ 目录 2026-09-24 重开: 三链通用清场脚本 (起链前/收场后调用; gz 家族模式与
# unistackbot_gazebo 的 gz_clean.sh 镜像, 模式安全规则见脚本头)
install(PROGRAMS scripts/chain_clean.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
