cmake_minimum_required(VERSION 3.8)
project(unistackbot_description)

find_package(ament_cmake REQUIRED)

# 形态分类目录 (2026-10-14): arms=固定基座 / mobile_manipulators=移动底盘 / humanoids=人形
install(DIRECTORY arms mobile_manipulators humanoids config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
