cmake_minimum_required(VERSION 3.8)
project(unistackbot_algorithm)

# colcon 不传 BUILD_TYPE 时默认无优化, 求解器热路径不可接受
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

# ============================================================================
# SDK 包 (2026-11-01 两包拆分定稿; 2026-11-02 自包含收口):
#   契约头 (纯 .hpp): ik_solver / ik_result / whole_body / base_kinematics
#   可选共享库: urdf_fk (FK/雅可比, 想用就用)
#   零本仓包依赖 (IkResult/Redundancy 词汇已从 interface 迁入)——
#   算法团队最小闭包 = 本包一个; 不含: IK/WB 实现 (住 Pkg 2)、
#   gravity_torque、collision (SDK 不替算法选库)
# ============================================================================

add_library(${PROJECT_NAME} SHARED
	urdf_fk/urdf_fk.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>
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
# 契约头 + SDK 库头 整体安装 (只装 .hpp, test/ 不进 install)
install(DIRECTORY urdf_fk ik_solver ik_result whole_body base_kinematics
	DESTINATION include
	FILES_MATCHING PATTERN "*.hpp"
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
# 传递依赖导出 (2026-11-02 外部包 dogfood 实锤): imported target 的 INTERFACE 带着
# urdf/urdfdom/KDL 链接依赖出仓, 不导出 dependencies 则外部消费方 find_package 缺传递包
ament_export_dependencies(orocos_kdl_vendor orocos_kdl urdf urdfdom kdl_parser)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
