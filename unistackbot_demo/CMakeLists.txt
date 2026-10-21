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
find_package(ament_index_cpp REQUIRED)
find_package(unistackbot_interface REQUIRED)
# Ubuntu libtinyxml2-dev 不带 CMake config; tinyxml2_vendor 提供 Find 模块 (同 controller 包)
list(APPEND CMAKE_MODULE_PATH /opt/ros/humble/share/tinyxml2_vendor/cmake/Modules)
find_package(TinyXML2 REQUIRED)

# 关节满速流 demo (命令域③: = 控制频率, 透传路径) —— C++ 实现的理由: python rclpy
# 发布 500Hz 全是抖动, 满速档必须是真调度; 自校准频率 (机型/链无关)
add_executable(demo_joint_fullrate src/demo_joint_fullrate.cpp)
target_compile_features(demo_joint_fullrate PUBLIC cxx_std_17)
ament_target_dependencies(demo_joint_fullrate
  rclcpp rcl_interfaces geometry_msgs sensor_msgs unistackbot_interface ament_index_cpp
)
target_link_libraries(demo_joint_fullrate ${TINYXML2_LIBRARY})

install(TARGETS demo_joint_fullrate
  DESTINATION lib/${PROJECT_NAME}
)

# 末端位姿流 (命令域①) / 关节慢流 (命令域②) / 键盘遥操作 (命令域④交互, 笛卡尔+关节两域) —— python
install(PROGRAMS
  scripts/demo_cartesian.py
  scripts/demo_motion.py
  scripts/demo_teleop_key.py
  scripts/demo_teleop_joint.py
  scripts/demo_points_playback.py
  scripts/minimal/arms/minimal_joint.py
  scripts/minimal/arms/minimal_cartesian.py
  DESTINATION lib/${PROJECT_NAME}
)

# 机型专属 demo (2026-10-12 约定): scripts/<robot>_arm/ 每机一文件夹, 文件名带
# 机型前缀 —— install 平铺进 lib/ (ros2 run 不认子目录, 重名即冲突)。
# r1_7a_arm = 首件 (关节+笛卡尔); r1_pro_arm = 四件式首例 (双 JS 实例+@js 通配切换)。
install(PROGRAMS
  scripts/r1_7a_arm/r1_7a_demo_joint.py
  scripts/r1_7a_arm/r1_7a_demo_cartesian.py
  scripts/r1_pro_arm/r1_pro_demo_joint.py
  scripts/r1_pro_arm/r1_pro_demo_cartesian.py
  DESTINATION lib/${PROJECT_NAME}
)
# stationlite_arm: 真机轨迹回放 demo (2026-10-20); 点位资产 json 装 share, 脚本运行时双路径查找
install(PROGRAMS scripts/stationlite_arm/stationlite_demo_replay.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES scripts/stationlite_arm/stationlite_joint_cycle_points.json
  DESTINATION share/${PROJECT_NAME}/stationlite_arm
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
