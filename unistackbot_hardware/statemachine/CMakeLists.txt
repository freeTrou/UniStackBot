cmake_minimum_required(VERSION 3.8)
project(unistackbot_statemachine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_GNUCXX)
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(unistackbot_protocol REQUIRED)

# 状态机翻译轴 (2026-09-24 拆包): StateTranslator 父类 + UnitreeImTranslator 子类 + 工厂。
# 零 ROS 依赖, g++ 直编离线测 (见 test/)。加新翻译器 = state_translator_factory.cpp 加一行。
add_library(${PROJECT_NAME}
  src/unitree_im_translator.cpp
  src/state_translator_factory.cpp
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
