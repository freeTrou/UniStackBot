cmake_minimum_required(VERSION 3.8)
project(unistackbot_robot_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(unistackbot_protocol REQUIRED)
find_package(unistackbot_statemachine REQUIRED)
find_package(unistackbot_bus REQUIRED)
find_package(unistackbot_udp_board REQUIRED)

# RobotSystem = 通用总线 SystemInterface 适配器 (组合根, 2026-11-02):
# "全工程唯一碰 ROS 的总线路径"的单数实现——一条代码服务所有 MasterBase 总线,
# 总线差异住配置(URDF)与总线包; registerKnownBuses 组合根注册表(加总线=加一行)。
add_library(${PROJECT_NAME} SHARED src/robot_system.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common
)
ament_target_dependencies(${PROJECT_NAME}
	hardware_interface pluginlib rclcpp rclcpp_lifecycle
	unistackbot_protocol unistackbot_statemachine unistackbot_bus unistackbot_udp_board
)
# 工厂注册符号 (老式 ament 包无导出目标: 经 LIBRARIES/LIBRARY_DIRS 变量取路径)
target_link_libraries(${PROJECT_NAME} ${unistackbot_udp_board_LIBRARIES})
target_link_directories(${PROJECT_NAME} PRIVATE ${unistackbot_udp_board_LIBRARY_DIRS})

install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
pluginlib_export_plugin_description_file(hardware_interface robot_system.xml)
install(FILES robot_system.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
