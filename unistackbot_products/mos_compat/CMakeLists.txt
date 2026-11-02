cmake_minimum_required(VERSION 3.8)
project(mos_compat)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(unistackbot_interface REQUIRED)

# MosCompat 门面节点 (Phase 2, 2026-10-30):
# /sdk/arm/joint_servo_cmd → /js_arms/command (JointCommand CSP)
# /sdk/arm/left+right_cartesian_servo_cmd → /whole_body_controller/command (PoseArray)
# /sdk/set_control_mode → controller switch (js↔whole_body)
add_executable(mos_compat_node src/mos_compat_node.cpp)
target_compile_features(mos_compat_node PUBLIC cxx_std_17)
ament_target_dependencies(mos_compat_node
	rclcpp sensor_msgs geometry_msgs std_msgs std_srvs tf2_ros tf2_geometry_msgs controller_manager_msgs unistackbot_interface
)
install(TARGETS mos_compat_node
	DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
