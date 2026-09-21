cmake_minimum_required(VERSION 3.8)
project(unistackbot_mujoco)

# MuJoCo 集成: 纯 launch 胶水 (控制器管理器宿主 = mujoco_ros2_control 定制节点,
# 引擎/物理线程/渲染都在该节点进程内, 本包零 C++)。
# MJCF 资产生成与策展在 description 包 arms/<robot>/mujoco/ (README 有再生命令)。

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
