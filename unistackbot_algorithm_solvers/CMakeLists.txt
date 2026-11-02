cmake_minimum_required(VERSION 3.8)
project(unistackbot_algorithm_solvers)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(unistackbot_algorithm REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)

# 显式 include 路径 (same workspace monorepo)
set(SDK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_algorithm)
set(COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../unistackbot_common)

# IK 源文件 (含 IKFast 生成物, 直接编进插件库)
set(IK_SOURCES
	analytic_piper/analytic_piper.cpp
	analytic_piper/piper_ikfast.cpp
	analytic_stationlite/analytic_stationlite.cpp
	analytic_stationlite/analytic_stationlite_ikfast.cpp
)
set_source_files_properties(analytic_piper/piper_ikfast.cpp analytic_stationlite/analytic_stationlite_ikfast.cpp PROPERTIES
	COMPILE_OPTIONS "-Wno-ignored-qualifiers;-Wno-pedantic;-Wno-unused-parameter;-DIKFAST_CLIBRARY;-DIKFAST_NO_MAIN")
set_source_files_properties(analytic_piper/piper_ikfast.cpp PROPERTIES COMPILE_DEFINITIONS "IKFAST_NAMESPACE=piper_ikfast")
set_source_files_properties(analytic_stationlite/analytic_stationlite_ikfast.cpp PROPERTIES COMPILE_DEFINITIONS "IKFAST_NAMESPACE=stationlite_ikfast")

option(UNISTACKBOT_BUILD_TRAC_IK "TRAC-IK (rclcpp+nlopt)" ON)

# ---- IK 插件库 (源文件直接编入, 单 .so) ----
add_library(ik_solvers SHARED
	ik_plugins/ik_plugins.cpp
	${IK_SOURCES}
	base_kinematics/base_kinematics.cpp
	base_kinematics/n_steer_kinematics.cpp
)
target_compile_features(ik_solvers PUBLIC cxx_std_17)
target_include_directories(ik_solvers PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}
	${SDK_INCLUDE_DIR}

	${COMMON_INCLUDE_DIR}
)
ament_target_dependencies(ik_solvers
	pluginlib urdf urdfdom orocos_kdl kdl_parser
)
target_link_libraries(ik_solvers
	unistackbot_algorithm::unistackbot_algorithm
	Eigen3::Eigen
)

if(UNISTACKBOT_BUILD_TRAC_IK)
	find_package(rclcpp REQUIRED)
	find_library(NLOPT_LIB nlopt REQUIRED)

	add_library(unistackbot_trac_ik SHARED
		trac_ik_solver/trac_ik_solver.cpp
		trac_ik_solver/vendor/trac_ik/trac_ik.cpp
		trac_ik_solver/vendor/trac_ik/nlopt_ik.cpp
		trac_ik_solver/vendor/trac_ik/kdl_tl.cpp
	)
	set_source_files_properties(
		trac_ik_solver/vendor/trac_ik/trac_ik.cpp
		trac_ik_solver/vendor/trac_ik/nlopt_ik.cpp
		trac_ik_solver/vendor/trac_ik/kdl_tl.cpp
		PROPERTIES COMPILE_OPTIONS "-Wno-pedantic;-Wno-unused-parameter;-Wno-sign-compare")
	target_include_directories(unistackbot_trac_ik PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_SOURCE_DIR}/trac_ik_solver/vendor
		${SDK_INCLUDE_DIR}
	)
	ament_target_dependencies(unistackbot_trac_ik
		rclcpp urdf urdfdom orocos_kdl kdl_parser
	)
	target_link_libraries(unistackbot_trac_ik
		${NLOPT_LIB} Eigen3::Eigen
		unistackbot_algorithm::unistackbot_algorithm
	)
	install(TARGETS unistackbot_trac_ik
		ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin
	)
	install(DIRECTORY trac_ik_solver DESTINATION include FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
	target_sources(ik_solvers PRIVATE ik_plugins/ik_plugins_trac_ik.cpp)
	ament_target_dependencies(ik_solvers rclcpp)
	target_link_libraries(ik_solvers unistackbot_trac_ik)
	pluginlib_export_plugin_description_file(unistackbot_algorithm ik_plugins/ik_plugins_trac_ik.xml)
endif()

install(TARGETS ik_solvers
	ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin
)
pluginlib_export_plugin_description_file(unistackbot_algorithm ik_plugins/ik_plugins.xml)

# ---- WholeBody 插件库 ----
add_library(whole_body_solvers SHARED
	ik_plugins/whole_body_plugins.cpp
	whole_body/upper_body_hqp_mos_compile.cpp   # 编译哨兵: 主模板进常规构建防烂
)
target_compile_features(whole_body_solvers PUBLIC cxx_std_17)
target_include_directories(whole_body_solvers PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}
	${SDK_INCLUDE_DIR}
	${SDK_INCLUDE_DIR}/whole_body

	${COMMON_INCLUDE_DIR}
)
ament_target_dependencies(whole_body_solvers
	pluginlib urdf urdfdom orocos_kdl kdl_parser
)
target_link_libraries(whole_body_solvers
	unistackbot_algorithm::unistackbot_algorithm
	Eigen3::Eigen
)
install(TARGETS whole_body_solvers
	ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin
)
pluginlib_export_plugin_description_file(unistackbot_algorithm ik_plugins/whole_body_plugins.xml)

# ---- 安装 ----
install(FILES ik_plugins/ik_plugins.xml DESTINATION share/${PROJECT_NAME}/ik_plugins)
if(UNISTACKBOT_BUILD_TRAC_IK)
	install(FILES ik_plugins/ik_plugins_trac_ik.xml DESTINATION share/${PROJECT_NAME}/ik_plugins)
endif()
install(FILES ik_plugins/whole_body_plugins.xml DESTINATION share/${PROJECT_NAME}/ik_plugins)
install(DIRECTORY whole_body analytic_piper analytic_stationlite analytic_common DESTINATION include FILES_MATCHING PATTERN "*.hpp")

ament_package()
