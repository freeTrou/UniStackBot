cmake_minimum_required(VERSION 3.8)
project(unistackbot_bus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(unistackbot_protocol REQUIRED)
find_package(unistackbot_statemachine REQUIRED)

# 总线主站轴 (2026-09-24 立轴): MasterBase 父类 (master.hpp 五要素最小集代码化,
# 纯交换零线程可见) + 工厂。交换类型自持 (BusCommand/BusState) —— 零 ROS 链:
# interface ← protocol ← statemachine ← bus。
# 子类: SerialMaster 随切片1步4 (串口泵骨架) 落地并在 master_factory.cpp 注册 "serial"。
add_library(${PROJECT_NAME}
  src/master_factory.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  unistackbot_protocol::unistackbot_protocol
  unistackbot_statemachine::unistackbot_statemachine
)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(unistackbot_protocol unistackbot_statemachine)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
