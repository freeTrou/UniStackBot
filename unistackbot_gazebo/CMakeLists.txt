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

# Gazebo (Fortress) 适配器为可选组件: 找不到 ign 依赖时跳过 (不影响 world/launch 的安装)
find_package(ignition-transport11 QUIET)
find_package(ignition-msgs8 QUIET)
if(ignition-transport11_FOUND AND ignition-msgs8_FOUND)
	message(STATUS "sim_control gz 适配器: 启用")
else()
	message(STATUS "sim_control gz 适配器: 未找到 ign 依赖, 跳过")
endif()

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/gz_clean.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(ignition-transport11_FOUND AND ignition-msgs8_FOUND)
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
	)
	target_link_libraries(sim_control_gz_node
		ignition-transport11::ignition-transport11
		ignition-msgs8::ignition-msgs8
	)
	install(TARGETS sim_control_gz_node
		DESTINATION lib/${PROJECT_NAME}
	)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
