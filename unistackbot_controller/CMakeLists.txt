cmake_minimum_required(VERSION 3.8)
project(unistackbot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(unistackbot_interface REQUIRED)
find_package(kdl_parser REQUIRED)

# 形态盲运动学库: URDF -> KDL 链 -> FK + 雅可比 (无机型知识, 机型只是数据)
add_library(${PROJECT_NAME} SHARED
  src/urdf_fk.cpp
  src/dls_ik.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp urdf urdfdom orocos_kdl kdl_parser
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# FK 命令行工具 (调试 + TF 对拍)
add_executable(fk_tool src/fk_tool.cpp)
target_compile_features(fk_tool PUBLIC cxx_std_17)
target_link_libraries(fk_tool ${PROJECT_NAME})
ament_target_dependencies(fk_tool rclcpp)

# IK 命令行工具 (调试 + RViz 演示)
add_executable(ik_tool src/ik_tool.cpp)
target_compile_features(ik_tool PUBLIC cxx_std_17)
target_link_libraries(ik_tool ${PROJECT_NAME})
ament_target_dependencies(ik_tool rclcpp unistackbot_interface)
target_include_directories(ik_tool PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
)

install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS fk_tool ik_tool
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp orocos_kdl kdl_parser unistackbot_interface)
ament_package()
