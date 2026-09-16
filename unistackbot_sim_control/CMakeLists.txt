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
	unistackbot_interface
)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
	find_package(${Dependency} REQUIRED)
endforeach()
rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/SetJointState.srv"
)

add_library(sim_control_hardware SHARED
	src/sim_control_hardware.cpp
	src/backend_kinematic.cpp
	src/backend_threaded.cpp
)
target_compile_features(sim_control_hardware PUBLIC cxx_std_17)
target_include_directories(sim_control_hardware PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common>   # 交换原语 (monorepo 内直引)
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>   # /sim_control 契约头 (同上)
	$<INSTALL_INTERFACE:include>
)
ament_target_dependencies(sim_control_hardware
	${THIS_PACKAGE_INCLUDE_DEPENDS}
)


# 同包 srv 类型支持 (intra-package 规范写法)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(sim_control_hardware "${cpp_typesupport_target}")

pluginlib_export_plugin_description_file(hardware_interface hardware_plugin.xml)

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
