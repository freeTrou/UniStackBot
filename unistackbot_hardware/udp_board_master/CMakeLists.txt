cmake_minimum_required(VERSION 3.8)
project(unistackbot_udp_board)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# UDP 电控板仿真侧件 (2026-10-30, Phase 0B; mos_hal 融入 bus 第四总线第一步):
# 本包当前零外部依赖 (帧编解码 + FakeBoard 纯 C++); 主站实现 (MasterBase/泵/状态机)
# 进后续批次, 届时补 unistackbot_bus/protocol/statemachine 依赖。
add_library(${PROJECT_NAME}
	board_protocol.cpp
	fake_board.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
	EXPORT export_${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)
install(DIRECTORY include/
	DESTINATION include
)
ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})

add_executable(fake_board_daemon src/fake_board_daemon.cpp)
target_compile_features(fake_board_daemon PUBLIC cxx_std_17)
target_link_libraries(fake_board_daemon ${PROJECT_NAME})
install(TARGETS fake_board_daemon DESTINATION lib/${PROJECT_NAME})

# 单元测试 g++ 直编 (零 ROS; hardware 域先例): bash test/run_udp_board_test.sh
# ---- SystemInterface 插件 (Phase 3, 2026-10-30): ros2_control 硬件组件 ----
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(udp_board_system SHARED
	src/udp_board_system.cpp
)
target_compile_features(udp_board_system PUBLIC cxx_std_17)
target_include_directories(udp_board_system PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)
ament_target_dependencies(udp_board_system
	hardware_interface pluginlib rclcpp rclcpp_lifecycle
)
target_link_libraries(udp_board_system ${PROJECT_NAME})

install(TARGETS udp_board_system
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)
pluginlib_export_plugin_description_file(hardware_interface udp_board_system.xml)
install(FILES udp_board_system.xml
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
