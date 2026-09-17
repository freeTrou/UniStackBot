cmake_minimum_required(VERSION 3.8)
project(unistackbot_controller)

# colcon 不传 BUILD_TYPE 时默认无优化 (-O0), 求解器热路径不可接受 (IK 迭代成本
# 实测差数倍)。RelWithDebInfo = -O2 -g, 与测试脚本 (g++ -O2) 同档。
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

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
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(kdl_parser REQUIRED)

# 形态盲运动学库: URDF -> KDL 链 -> FK + 雅可比 (无机型知识, 机型只是数据)
add_library(${PROJECT_NAME} SHARED
  src/urdf_fk.cpp
  src/dls_ik.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common>
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

# IK 交互演示节点 (RViz 6D 拖动标记)
add_executable(ik_demo_node src/ik_demo_node.cpp)
target_compile_features(ik_demo_node PUBLIC cxx_std_17)
target_link_libraries(ik_demo_node ${PROJECT_NAME})
ament_target_dependencies(ik_demo_node rclcpp unistackbot_interface interactive_markers visualization_msgs rclcpp_action control_msgs trajectory_msgs)
target_include_directories(ik_demo_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common>
)

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
install(TARGETS fk_tool ik_tool ik_demo_node
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
