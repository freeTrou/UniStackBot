cmake_minimum_required(VERSION 3.8)
project(mos_p20_description)

find_package(ament_cmake REQUIRED)

# 机型资产整目录安装 (meshes 105MB + urdf + config; original/ 死档案不装)
install(DIRECTORY urdf meshes config
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
