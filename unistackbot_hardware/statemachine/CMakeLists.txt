cmake_minimum_required(VERSION 3.8)
project(unistackbot_statemachine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_GNUCXX)
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(unistackbot_protocol REQUIRED)

# 状态机轴 (2026-10-10 重构归位, 同 protocol/bus 组织): NodeSafety 父类 (中立骨架,
# header-only) + UnitreeImSafety 子类 (MIT 语义) + 工厂。零 ROS 依赖, g++ 直编离线测
# (见 test/)。加新协议安全机 = node_safety_factory.cpp 注册表加一行。
add_library(${PROJECT_NAME}
  src/unitree_im_safety.cpp
  src/cia402_safety.cpp
  src/node_safety_factory.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} unistackbot_protocol::unistackbot_protocol)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(unistackbot_protocol)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
