cmake_minimum_required(VERSION 3.8)
project(unistackbot_mujoco)

# MuJoCo 集成: launch 胶水 + /sim_control 的 mujoco 适配器 (0d, 2026-09-21)。
# 控制器管理器宿主 = mujoco_ros2_control 定制节点 (引擎/物理线程/渲染在其进程内)。
# MJCF 资产生成与策展在 description 包 arms/<robot>/mujoco/ (README 有再生命令)。

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mujoco_ros2_control_msgs REQUIRED)
find_package(unistackbot_sim_control REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# /sim_control 契约的 mujoco 适配器 (契约消费者; 依赖方向 mujoco → sim_control, 同 gz)
add_executable(sim_control_mujoco_node src/sim_control_mujoco_node.cpp)
target_compile_features(sim_control_mujoco_node PUBLIC cxx_std_17)
target_include_directories(sim_control_mujoco_node PRIVATE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../unistackbot_common>              # ulog
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../unistackbot_interface/include>   # SimResult/kMaxJoints
)
ament_target_dependencies(sim_control_mujoco_node
	rclcpp
	mujoco_ros2_control_msgs
	unistackbot_sim_control
)
install(TARGETS sim_control_mujoco_node
	DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
