cmake_minimum_required(VERSION 3.8)
project(unistackbot_protocol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 纯协议库 (2026-09-24 开包+切片1落地): 协议内芯(一芯三面)+中立机(粗态)+工厂注册表。
# 零 ROS 依赖——核心文件纯 std, g++ 直编离线测 (同 ulog/algorithm 先例, 见 test/)。
# 总线骨架 (unistackbot_hardware) 消费本包; 依赖单向 protocol←hardware。
# 加新协议 = src/protocol_factory.cpp 注册表加一行 (设计 §2.1)。
add_library(${PROJECT_NAME}
  src/unitree_im_core.cpp
  src/protocol_factory.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
