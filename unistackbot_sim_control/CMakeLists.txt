cmake_minimum_required(VERSION 3.8)
project(unistackbot_sim_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
	rclcpp
	hardware_interface
	pluginlib
	std_srvs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
	find_package(${Dependency} REQUIRED)
endforeach()
find_package(rosidl_default_generators REQUIRED)

# Gazebo (Fortress) 适配器为可选组件: 找不到 ign 依赖时跳过, 不影响插件本体
find_package(ignition-transport11 QUIET)
find_package(ignition-msgs8 QUIET)
if(ignition-transport11_FOUND AND ignition-msgs8_FOUND)
	set(HAVE_GZ_ADAPTER TRUE)
	message(STATUS "Gazebo Fortress 适配器: 启用")
else()
	message(STATUS "Gazebo Fortress 适配器: 未找到 ign 依赖, 跳过")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/SetJointState.srv"
)

add_library(sim_control_hardware SHARED
	src/sim_control_hardware.cpp
	src/backend_kinematic.cpp
	src/sim_control_server.cpp
)
target_compile_features(sim_control_hardware PUBLIC cxx_std_17)
target_include_directories(sim_control_hardware PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sim_control_hardware
	${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# 同包 srv 类型支持
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(sim_control_hardware "${cpp_typesupport_target}")

pluginlib_export_plugin_description_file(hardware_interface hardware_plugin.xml)

if(HAVE_GZ_ADAPTER)
	add_executable(sim_control_gz_node src/sim_control_gz_node.cpp)
	target_compile_features(sim_control_gz_node PUBLIC cxx_std_17)
	target_include_directories(sim_control_gz_node PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	)
	ament_target_dependencies(sim_control_gz_node rclcpp std_srvs)
	target_link_libraries(sim_control_gz_node
		sim_control_hardware
		"${cpp_typesupport_target}"
		ignition-transport11::ignition-transport11
		ignition-msgs8::ignition-msgs8
	)
	install(TARGETS sim_control_gz_node
		DESTINATION lib/${PROJECT_NAME}
	)
endif()

install(TARGETS sim_control_hardware EXPORT export_${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)
install(DIRECTORY include/
	DESTINATION include
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(sim_control_hardware)

ament_package()
