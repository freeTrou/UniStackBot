cmake_minimum_required(VERSION 3.8)
project(unistackbot_gazebo)

# Gazebo 集成: world + launch + 清残留脚本 + gz 后端适配器 (/sim_control 契约的
# gz 实现; 契约住 sim_control 包, 类型住 interface 包 —— 2026-09-17 定稿)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unistackbot_interface REQUIRED)
find_package(unistackbot_sim_control REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/gz_clean.sh
  DESTINATION lib/${PROJECT_NAME}
)

# gz 后端适配器 (契约消费者: 依赖 sim_control 的 /sim_control 契约 + interface 的类型)
add_executable(sim_control_gz_node src/sim_control_gz_node.cpp)
target_compile_features(sim_control_gz_node PUBLIC cxx_std_17)
target_include_directories(sim_control_gz_node PRIVATE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../unistackbot_common>              # ulog
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../unistackbot_interface/include>   # SimResult/kMaxJoints (纯头直引)
)
ament_target_dependencies(sim_control_gz_node
	rclcpp
	std_srvs
	unistackbot_interface
	unistackbot_sim_control
	ros_gz_interfaces
)
install(TARGETS sim_control_gz_node
	DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
