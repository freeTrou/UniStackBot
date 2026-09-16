cmake_minimum_required(VERSION 3.8)
project(unistackbot_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 接口定义目录:
#   msg/   话题与服务字段定义 (rosidl 生成 C++/Python 类型)
#   action/ (预留)
# 纯 C++ 共享头 (非 rosidl) 放 include/unistackbot_interface/, 由消费方加 include 路径直引
set(INTERFACE_FILES
	# msg/ 待第一批跨进程类型 (RL ingress / state 出口) 出现
)
if(INTERFACE_FILES)
	rosidl_generate_interfaces(${PROJECT_NAME} ${INTERFACE_FILES})
	ament_export_dependencies(rosidl_default_runtime)
endif()

install(DIRECTORY include/
	DESTINATION include
)
# 契约头对消费方可见 (rosidl 生成头由 rosidl 管道导出, 纯 C++ 头需显式导出)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_srvs)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
