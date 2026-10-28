cmake_minimum_required(VERSION 3.8)
project(unistackbot_canfd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(unistackbot_protocol REQUIRED)
find_package(unistackbot_statemachine REQUIRED)
find_package(unistackbot_bus REQUIRED)

# SocketCAN 总线骨架 (2026-10-28, MasterBase 第二实现; 设计 docs/bus/socketcan_master_design.md):
# 单类 1+1 (CAN 无 Framer / socket 住 Impl / vcan 即测试缝)。经典+FD 双模
# (frame_len 分型)。依赖 common 组件 (SpLatest/rt_tune) monorepo 直引 (同 serial 先例)。
add_library(${PROJECT_NAME}
  src/canfd_master.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../unistackbot_common>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  unistackbot_protocol::unistackbot_protocol
  unistackbot_statemachine::unistackbot_statemachine
  unistackbot_bus::unistackbot_bus
  pthread
)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(unistackbot_protocol unistackbot_statemachine unistackbot_bus)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
