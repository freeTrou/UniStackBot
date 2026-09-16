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
find_package(kdl_parser REQUIRED)

# 形态盲运动学库: URDF -> KDL 链 -> FK + 雅可比 (无机型知识, 机型只是数据)
add_library(${PROJECT_NAME} SHARED
  src/urdf_fk.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp urdf urdfdom orocos_kdl kdl_parser
)

# FK 命令行工具 (调试 + TF 对拍)
add_executable(fk_tool src/fk_tool.cpp)
target_compile_features(fk_tool PUBLIC cxx_std_17)
target_link_libraries(fk_tool ${PROJECT_NAME})
ament_target_dependencies(fk_tool rclcpp)

install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS fk_tool
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
ament_export_dependencies(rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros)
ament_package()
