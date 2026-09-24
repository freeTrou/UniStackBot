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
find_package(Eigen3 REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)
# Ubuntu libtinyxml2-dev 不带 CMake config; tinyxml2_vendor 提供 Find 模块
list(APPEND CMAKE_MODULE_PATH /opt/ros/humble/share/tinyxml2_vendor/cmake/Modules)
find_package(TinyXML2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unistackbot_interface REQUIRED)
find_package(unistackbot_algorithm REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)

# 算法库 (urdf_fk/dls_ik) 已拆至 unistackbot_algorithm (2026-09-17);
# 本包 = 控制器集成包: 每控制器一个自包含文件夹 + tools/ 工具归拢。

# 笛卡尔流式运动控制器 (P1.5): IK 算法上环的宿主
add_library(cartesian_motion_controller SHARED
  cartesian_motion_controller/cartesian_motion_controller.cpp
)
target_compile_features(cartesian_motion_controller PUBLIC cxx_std_17)
target_include_directories(cartesian_motion_controller PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common/ruckig/include>
)
# §16.6: CM 输出级 OtgStream —— ruckig 13 个 .cpp 参与编译 (组件非单头文件)
file(GLOB CM_RUCKIG_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common/ruckig/src/ruckig/*.cpp)
target_sources(cartesian_motion_controller PRIVATE ${CM_RUCKIG_SOURCES})
ament_target_dependencies(cartesian_motion_controller
  rclcpp controller_interface realtime_tools pluginlib ament_index_cpp
  geometry_msgs unistackbot_interface unistackbot_algorithm
)
target_link_libraries(cartesian_motion_controller ${TINYXML2_LIBRARY})
# 类目名 = 基类包名 controller_interface (实测: ros2_control 文档写法在本 Humble
# 不生效; sim_control 用 hardware_interface 同构可证)
pluginlib_export_plugin_description_file(controller_interface
  cartesian_motion_controller/cartesian_motion_controller.xml
)
pluginlib_export_plugin_description_file(controller_interface
  joint_stream_controller/joint_stream_controller.xml
)
# 注意: 该宏每次调用只注册一个 xml (多参数被静默忽略 —— 2026-09-21 实测踩坑)
pluginlib_export_plugin_description_file(controller_interface
  ee_state_broadcaster/ee_state_broadcaster.xml
)
# OTG 门 (P1, 设计 §16.4 路线 A): 笛卡尔终点 → IK → OtgStream 关节整形流
file(GLOB OTG_GATE_RUCKIG ${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common/ruckig/src/ruckig/*.cpp)
add_library(otg_gate_controller SHARED
  otg_gate/otg_gate_controller.cpp
  ${OTG_GATE_RUCKIG}
)
target_compile_features(otg_gate_controller PUBLIC cxx_std_17)
target_include_directories(otg_gate_controller PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common/ruckig/include>
)
ament_target_dependencies(otg_gate_controller
  rclcpp controller_interface realtime_tools pluginlib ament_index_cpp
  geometry_msgs sensor_msgs unistackbot_interface unistackbot_algorithm
)
target_link_libraries(otg_gate_controller ${TINYXML2_LIBRARY})
pluginlib_export_plugin_description_file(controller_interface
  otg_gate/otg_gate_controller.xml
)
# 关节级 topic 流式控制器 (取代 JTC action 层, 2026-09-18 裁决)
file(GLOB RUCKIG_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common/ruckig/src/ruckig/*.cpp)
add_library(joint_stream_controller SHARED
  joint_stream_controller/joint_stream_controller.cpp
  ${RUCKIG_SOURCES}
)
target_compile_features(joint_stream_controller PUBLIC cxx_std_17)
target_include_directories(joint_stream_controller PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common/ruckig/include>
)
ament_target_dependencies(joint_stream_controller
  rclcpp controller_interface realtime_tools pluginlib ament_index_cpp
  geometry_msgs unistackbot_interface
)
target_link_libraries(joint_stream_controller ${TINYXML2_LIBRARY})

# EE 位姿独立反馈流 (仿真测试方案批次2, 2026-09-21): 只读控制器, 与 JS/CM 共存
add_library(ee_state_broadcaster SHARED
  ee_state_broadcaster/ee_state_broadcaster.cpp
)
target_compile_features(ee_state_broadcaster PUBLIC cxx_std_17)
target_include_directories(ee_state_broadcaster PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common>
)
ament_target_dependencies(ee_state_broadcaster
  rclcpp controller_interface realtime_tools pluginlib
  geometry_msgs unistackbot_interface unistackbot_algorithm
)
target_link_libraries(ee_state_broadcaster ${TINYXML2_LIBRARY})
install(TARGETS ee_state_broadcaster
  DESTINATION lib
)
install(FILES ee_state_broadcaster/ee_state_broadcaster.xml
  DESTINATION share/${PROJECT_NAME}/ee_state_broadcaster
)

# FK 命令行工具 (调试 + TF 对拍)
add_executable(fk_tool tools/fk_tool.cpp)
target_compile_features(fk_tool PUBLIC cxx_std_17)
ament_target_dependencies(fk_tool rclcpp unistackbot_algorithm)
target_include_directories(fk_tool PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
)

# IK 交互演示节点 (RViz 6D 拖动标记)
add_executable(ik_demo_node tools/ik_demo_node.cpp)
target_compile_features(ik_demo_node PUBLIC cxx_std_17)
ament_target_dependencies(ik_demo_node rclcpp unistackbot_interface interactive_markers visualization_msgs geometry_msgs)
target_include_directories(ik_demo_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common>
)

# IK 命令行工具 (调试 + RViz 演示)
add_executable(ik_tool tools/ik_tool.cpp)
target_compile_features(ik_tool PUBLIC cxx_std_17)
ament_target_dependencies(ik_tool rclcpp unistackbot_interface unistackbot_algorithm)
target_include_directories(ik_tool PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
)

install(TARGETS fk_tool ik_tool ik_demo_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS cartesian_motion_controller joint_stream_controller otg_gate_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
# pluginlib 环境钩子按"包内相对路径"解析 xml —— 安装必须保持目录结构, 不能拍平
install(FILES cartesian_motion_controller/cartesian_motion_controller.xml
  DESTINATION share/${PROJECT_NAME}/cartesian_motion_controller
)
install(FILES otg_gate/otg_gate_controller.xml
  DESTINATION share/${PROJECT_NAME}/otg_gate
)
install(FILES joint_stream_controller/joint_stream_controller.xml
  DESTINATION share/${PROJECT_NAME}/joint_stream_controller
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(cartesian_motion_controller joint_stream_controller)
ament_export_dependencies(controller_interface unistackbot_algorithm)
ament_package()
