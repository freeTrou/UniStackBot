cmake_minimum_required(VERSION 3.16)
project(unistackbot_tool)

find_package(ament_cmake REQUIRED)

# 纯脚本工具包 (零编译): 录制器 (bash 包 ros2 bag) + 点位转换 (python stdlib)
install(PROGRAMS
  scripts/record_verify.sh
  scripts/convert_points_v0_v1.py
  scripts/check_effort_flips.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
