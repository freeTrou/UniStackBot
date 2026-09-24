cmake_minimum_required(VERSION 3.16)
project(unistackbot_demo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(unistackbot_interface REQUIRED)
# Ubuntu libtinyxml2-dev 不带 CMake config; tinyxml2_vendor 提供 Find 模块 (同 controller 包)
list(APPEND CMAKE_MODULE_PATH /opt/ros/humble/share/tinyxml2_vendor/cmake/Modules)
find_package(TinyXML2 REQUIRED)

# 关节满速流 demo (命令域③: = 控制频率, 透传路径) —— C++ 实现的理由: python rclpy
# 发布 500Hz 全是抖动, 满速档必须是真调度; 自校准频率 (机型/链无关)
add_executable(demo_joint_fullrate src/demo_joint_fullrate.cpp)
target_compile_features(demo_joint_fullrate PUBLIC cxx_std_17)
ament_target_dependencies(demo_joint_fullrate
  rclcpp rcl_interfaces geometry_msgs sensor_msgs unistackbot_interface
)
target_link_libraries(demo_joint_fullrate ${TINYXML2_LIBRARY})

install(TARGETS demo_joint_fullrate
  DESTINATION lib/${PROJECT_NAME}
)

# 末端位姿流 (命令域①) 与 关节慢流 (命令域②) demo —— python 点流
install(PROGRAMS
  scripts/demo_cartesian.py
  scripts/demo_motion.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
