cmake_minimum_required(VERSION 3.8)
project(unistackbot_algorithm)

# colcon 不传 BUILD_TYPE 时默认无优化 (-O0), 求解器热路径不可接受 (IK 迭代成本
# 实测差数倍)。RelWithDebInfo = -O2 -g, 与测试脚本 (g++ -O2) 同档。
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(unistackbot_interface REQUIRED)

# 纯算法库 (2026-09-17 自 unistackbot_controller 拆出): 运动学解算 + 数值 IK。
# 目录 = 每算法一个文件夹 (与 unistackbot_common 组件三件套同构):
#   urdf_fk/  ik_solver/  dls_ik/  (后续新算法各起一个文件夹, 互不交叉)
# include 基 = 包根, 消费方 #include "dls_ik/dls_ik.hpp"。
# 零控制器生态依赖 —— 外部仓库/RL 训练侧只依赖本包即可复用算法。
add_library(${PROJECT_NAME} SHARED
	urdf_fk/urdf_fk.cpp
	dls_ik/dls_ik.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common>
	$<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}
	urdf urdfdom orocos_kdl kdl_parser
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# 头文件随算法文件夹整体安装 (只装 .hpp, test/ 与 .cpp 不进 install)
install(DIRECTORY urdf_fk ik_solver dls_ik
	DESTINATION include
	FILES_MATCHING PATTERN "*.hpp"
)
install(TARGETS ${PROJECT_NAME}
	EXPORT export_${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(orocos_kdl kdl_parser unistackbot_interface)
ament_package()
