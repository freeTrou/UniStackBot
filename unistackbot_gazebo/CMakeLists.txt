cmake_minimum_required(VERSION 3.8)
project(unistackbot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
# /sim_control 契约 (srv + SimCommand + SimControlServer, header-only)
find_package(unistackbot_interface REQUIRED)
# 世界控制服务类型: pause/resume/step 经 ros_gz_bridge 桥接的 ControlWorld
find_package(ros_gz_interfaces REQUIRED)

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/gz_clean.sh
  DESTINATION lib/${PROJECT_NAME}
)

# 纯 ROS 构建: 无 ign 编译依赖 (2026-09-16 重写, 世界控制走 launch 里
# parameter_bridge 桥接的 /world/<world>/control ROS 服务)
add_executable(sim_control_gz_node src/sim_control_gz_node.cpp)
target_compile_features(sim_control_gz_node PUBLIC cxx_std_17)
target_include_directories(sim_control_gz_node PRIVATE
	# ulog 公共组件头 + /sim_control 契约头 (monorepo 内直引, 与 sim_control 同型)
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
)
ament_target_dependencies(sim_control_gz_node
	rclcpp
	std_srvs
	unistackbot_interface
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
