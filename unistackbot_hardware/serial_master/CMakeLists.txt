cmake_minimum_required(VERSION 3.8)
project(unistackbot_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(unistackbot_protocol REQUIRED)
find_package(unistackbot_statemachine REQUIRED)
find_package(unistackbot_bus REQUIRED)

# 串口总线骨架 (2026-09-24 切片1步3-4): 五件套 IO 层 + SerialMaster (MasterBase 实现)。
# 依赖 common 组件 (SpLatest/rt_tune)——monorepo 内直引, 对外发布前需调整 (同 controller 先例)。
add_library(${PROJECT_NAME}
  src/framer.cpp
  src/termios_transport.cpp
  src/serial_master.cpp
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
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(unistackbot_protocol unistackbot_statemachine unistackbot_bus)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
