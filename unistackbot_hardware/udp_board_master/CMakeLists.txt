cmake_minimum_required(VERSION 3.8)
project(unistackbot_udp_board)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(unistackbot_protocol REQUIRED)
find_package(unistackbot_statemachine REQUIRED)
find_package(unistackbot_bus REQUIRED)

# UDP 电控板主站 (纯总线骨架零 ROS; 2026-11-02 组合上收 bringup/BusSystem):
# UdpMaster = MasterBase 第三实现 (总线骨架/IO 工程); 协议内芯住 unistackbot_protocol
# (board_protocol 聚合帧族), 状态机住 unistackbot_statemachine (BoardSafety)。
# common 组件 (DynamicSpLatest/rt_tune) monorepo 直引 (serial/canfd 先例)。
add_library(${PROJECT_NAME} SHARED
	src/udp_master.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/../../unistackbot_common
)
target_link_libraries(${PROJECT_NAME}
	unistackbot_protocol::unistackbot_protocol
	unistackbot_statemachine::unistackbot_statemachine
	unistackbot_bus::unistackbot_bus
	pthread
)
install(TARGETS ${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
)
install(DIRECTORY include/
	DESTINATION include
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})



add_executable(fake_board_daemon src/fake_board_daemon.cpp)
target_compile_features(fake_board_daemon PUBLIC cxx_std_17)
target_link_libraries(fake_board_daemon unistackbot_protocol::unistackbot_protocol)
install(TARGETS fake_board_daemon DESTINATION lib/${PROJECT_NAME})

ament_package()
