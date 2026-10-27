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
# pluginlib: ik_solvers 注册库的条件依赖 (2026-10-26 审核修 F2 —— 此前靠 urdf 传递
# 导出侥幸; QUIET = 显式可选: 纯无 ROS 核心库构建时跳过注册库)
find_package(pluginlib QUIET)

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
	analytic_stationlite/analytic_stationlite.cpp
	analytic_stationlite/analytic_stationlite_ikfast.cpp
	base_kinematics/base_kinematics.cpp
	base_kinematics/n_steer_kinematics.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_interface/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common>
	$<INSTALL_INTERFACE:include>
)
# piper_ikfast.cpp 为 vendored 生成物 (ikfast.h 固有警告) → 单文件关该警告;
# IKFAST_CLIBRARY: 函数以 extern "C" 导出 (与 analytic_piper.cpp 的声明链接一致)。
# 2026-10-26 审核修 F5: pi_ 前缀 + IKFAST_NAMESPACE 补齐 (此前 piper 版 C API 为全局
# 符号, 与 stationlite 生成物同库共存且埋外部 ikfast 插件符号插换隐患)
set_source_files_properties(analytic_piper/piper_ikfast.cpp PROPERTIES
	COMPILE_OPTIONS "-Wno-ignored-qualifiers;-Wno-pedantic;-Wno-unused-parameter;-DIKFAST_CLIBRARY;-DIKFAST_NO_MAIN;-DIKFAST_NAMESPACE=piper_ikfast"
)
# stationlite 生成物同款隔离 (sl_ 前缀 + stationlite_ikfast ns)。
# 隔离机制 (两生成物同一 ${PROJECT_NAME} target): IKFAST_NAMESPACE 隔 C++ 实体命名,
# extern "C" 的 C API 符号不受 ns 约束 → 逐族前缀 (pi_/sl_) 是 C API 唯一隔离手段。
# 外部团队 vendored ikfast 件必须同样自隔离 (见 ik_solver.hpp 接入配方)。
set_source_files_properties(analytic_stationlite/analytic_stationlite_ikfast.cpp PROPERTIES
	COMPILE_OPTIONS "-Wno-ignored-qualifiers;-Wno-pedantic;-Wno-unused-parameter;-DIKFAST_CLIBRARY;-DIKFAST_NO_MAIN;-DIKFAST_NAMESPACE=stationlite_ikfast"
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
install(DIRECTORY urdf_fk ik_solver dls_ik analytic_piper analytic_stationlite analytic_common base_kinematics
	DESTINATION include
	FILES_MATCHING PATTERN "*.hpp"
)

# ---- TRAC-IK 可选件子库 (方案 C, 2026-10-08) ----
# vendored traclabs/trac_ik rolling 分支 (BSD-3, 2026-09-29; + 本仓 Humble 兼容
# patch: urdf::Model 不存在 → urdfdom parseURDF)。回调线程专用 (IkSolver 契约 4
# b 类, 见 trac_ik_solver.hpp 豁免声明) —— rclcpp+nlopt 只挂本目标。
# 交叉编译核心库 / 无 ROS 环境: -DUNISTACKBOT_BUILD_TRAC_IK=OFF —— ik_solvers 注册库
# 照常构建 (dls/analytic 三件零 trac_ik 依赖, 2026-10-26 审核修 F1: 不再随本开关消失),
# trac_ik 注册件与 unistackbot_trac_ik 子库除外; 纯无 ROS 世界 (pluginlib 缺) 只编核心库。
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

# ---- IkSolver pluginlib 注册库 (缝③, 2026-10-22; 2026-10-26 审核修 F1 移出 trac_ik 开关) ----
# 条件 = pluginlib 可用 (ROS 侧), 与 UNISTACKBOT_BUILD_TRAC_IK 解耦:
#   dls/analytic_piper/analytic_stationlite 零 trac_ik 依赖 → OFF (nlopt 缺) 世界照常注册;
#   纯无 ROS 世界只编核心库 (本块跳过)。trac_ik 注册件以条件源并入同库 + 条件 xml。
# yaml ik_solver 值 = xml 的 class name (裸短名, 机型 yaml 零改动)。
if(pluginlib_FOUND)
	add_library(ik_solvers SHARED
		ik_plugins/ik_plugins.cpp
	)
	target_compile_features(ik_solvers PUBLIC cxx_std_17)
	ament_target_dependencies(ik_solvers
		pluginlib urdf urdfdom orocos_kdl kdl_parser
	)
	target_link_libraries(ik_solvers
		${PROJECT_NAME} Eigen3::Eigen
	)
	if(UNISTACKBOT_BUILD_TRAC_IK)
		target_sources(ik_solvers PRIVATE
			ik_plugins/ik_plugins_trac_ik.cpp
		)
		ament_target_dependencies(ik_solvers rclcpp)
		target_link_libraries(ik_solvers unistackbot_trac_ik)
	endif()
	install(TARGETS ik_solvers
		ARCHIVE DESTINATION lib
		LIBRARY DESTINATION lib
		RUNTIME DESTINATION bin
	)
	# 类目名 = 基类包名 = 本包 (controller_interface/hardware_interface 先例);
	# 宏每次调用只注册一个 xml —— 多参数被静默忽略; 子目录 xml 路径 (controller 同款)
	pluginlib_export_plugin_description_file(unistackbot_algorithm ik_plugins/ik_plugins.xml)
	if(UNISTACKBOT_BUILD_TRAC_IK)
		pluginlib_export_plugin_description_file(unistackbot_algorithm ik_plugins/ik_plugins_trac_ik.xml)
	endif()
else()
	message(STATUS "pluginlib 未找到: 跳过 ik_solvers 注册库 (纯无 ROS 核心库构建场景)")
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
