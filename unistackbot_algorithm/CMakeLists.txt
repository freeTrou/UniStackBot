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
#   urdf_fk/  ik_solver/  dls_ik/  analytic_piper/  trac_ik_solver/
# include 基 = 包根, 消费方 #include "dls_ik/dls_ik.hpp"。
#
# 方案 C (2026-10-08 用户裁决): 核心库零 ROS / 零 nlopt —— 交叉编译与 RT/非 ROS
# 复用面 (外部仓库/RL 训练侧/ARM 主站) 只消费本目标; TRAC-IK 是主控机回调线程
# 专用件, 住在下方可选子库 unistackbot_trac_ik, 其 rclcpp+nlopt 依赖不进核心。
add_library(${PROJECT_NAME} SHARED
	urdf_fk/urdf_fk.cpp
	dls_ik/dls_ik.cpp
	analytic_piper/analytic_piper.cpp
	analytic_piper/piper_ikfast.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common>
	$<INSTALL_INTERFACE:include>
)
# piper_ikfast.cpp 为 vendored 生成物 (ikfast.h 固有警告) → 单文件关该警告;
# IKFAST_CLIBRARY: 函数以 extern "C" 导出 (与 analytic_piper.cpp 的声明链接一致)
set_source_files_properties(analytic_piper/piper_ikfast.cpp PROPERTIES
	COMPILE_OPTIONS "-Wno-ignored-qualifiers;-Wno-pedantic;-Wno-unused-parameter;-DIKFAST_CLIBRARY;-DIKFAST_NO_MAIN"
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}
	urdf urdfdom orocos_kdl kdl_parser
)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

install(TARGETS ${PROJECT_NAME}
	EXPORT export_${PROJECT_NAME}
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)
# 头文件随算法文件夹整体安装 (只装 .hpp, test/ 与 .cpp 不进 install)
install(DIRECTORY urdf_fk ik_solver dls_ik analytic_piper
	DESTINATION include
	FILES_MATCHING PATTERN "*.hpp"
)

# ---- TRAC-IK 可选件子库 (方案 C, 2026-10-08) ----
# vendored traclabs/trac_ik rolling 分支 (BSD-3, 2026-09-29; + 本仓 Humble 兼容
# patch: urdf::Model 不存在 → urdfdom parseURDF)。回调线程专用 (IkSolver 契约 4
# b 类, 见 trac_ik_solver.hpp 豁免声明) —— rclcpp+nlopt 只挂本目标。
# 交叉编译核心库 / 无 ROS 环境: -DUNISTACKBOT_BUILD_TRAC_IK=OFF (此时勿编
# controller —— CM 的 trac_ik 分支与 trac_ik_solver 头不在)。
option(UNISTACKBOT_BUILD_TRAC_IK "TRAC-IK 求解器可选件 (rclcpp+nlopt, 回调线程专用)" ON)
if(UNISTACKBOT_BUILD_TRAC_IK)
	find_package(rclcpp REQUIRED)
	find_library(NLOPT_LIB nlopt REQUIRED)
	add_library(unistackbot_trac_ik SHARED
		trac_ik_solver/trac_ik_solver.cpp
		trac_ik_solver/vendor/trac_ik/trac_ik.cpp
		trac_ik_solver/vendor/trac_ik/nlopt_ik.cpp
		trac_ik_solver/vendor/trac_ik/kdl_tl.cpp
	)
	# 上游固有警告单文件关 (vendored 代码不动其内容, 警告面就地屏蔽)
	set_source_files_properties(
		trac_ik_solver/vendor/trac_ik/trac_ik.cpp
		trac_ik_solver/vendor/trac_ik/nlopt_ik.cpp
		trac_ik_solver/vendor/trac_ik/kdl_tl.cpp
		PROPERTIES COMPILE_OPTIONS "-Wno-pedantic;-Wno-unused-parameter;-Wno-sign-compare")
	target_include_directories(unistackbot_trac_ik PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/trac_ik_solver/vendor>
		$<INSTALL_INTERFACE:include>
	)
	ament_target_dependencies(unistackbot_trac_ik
		rclcpp urdf urdfdom orocos_kdl kdl_parser
	)
	target_link_libraries(unistackbot_trac_ik
		${NLOPT_LIB} Eigen3::Eigen ${PROJECT_NAME})
	install(TARGETS unistackbot_trac_ik
		EXPORT export_${PROJECT_NAME}
		ARCHIVE DESTINATION lib
		LIBRARY DESTINATION lib
		RUNTIME DESTINATION bin
	)
	# trac_ik_solver 适配层头 + vendor 头 (trac_ik/trac_ik.hpp 是适配层的公开面)
	install(DIRECTORY trac_ik_solver trac_ik_solver/vendor/trac_ik
		DESTINATION include
		FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
	)
endif()

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
if(UNISTACKBOT_BUILD_TRAC_IK)
	ament_export_libraries(unistackbot_trac_ik)
endif()
ament_export_dependencies(orocos_kdl kdl_parser unistackbot_interface)
ament_package()
