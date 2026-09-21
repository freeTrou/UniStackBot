from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

# 三链统一入口 (2026-09-21, 仿真测试方案 §2):
#   ros2 launch unistackbot_bringup sim.launch.py chain:=mock|gz|mujoco robot:=<机型> ...
# 只做编排+参数转发——三链各自的 launch 是规范实现 (控制台提示与差异见
# docs/sim_environment_and_test_plan.md §2)。未知 chain/robot fail-fast 列合法值。
_CHAINS = {
	'mock': ('unistackbot_bringup', 'control.launch.py', ['robot', 'use_rviz']),
	'gz': ('unistackbot_gazebo', 'ign.launch.py', ['robot', 'gui', 'use_rviz']),
	'mujoco': ('unistackbot_mujoco', 'mujoco.launch.py', ['robot', 'headless', 'use_rviz']),
}


def _launch_setup(context):
	chain = LaunchConfiguration('chain').perform(context)
	robot = LaunchConfiguration('robot').perform(context)

	if chain not in _CHAINS:
		raise RuntimeError(
			f"未知链 {chain!r}: 可选 {sorted(_CHAINS)} "
			f"(mock=kinematic / gz=Gazebo Fortress / mujoco=MuJoCo)")
	pkg, launch_file, arg_names = _CHAINS[chain]

	# 机型存在性 fail-fast (与各链 launch 同报错口径; 统一入口提前拦, 报错更近因)
	desc_share = get_package_share_directory('unistackbot_description')
	xacro_path = os.path.join(desc_share, 'arms', robot, 'urdf', f'{robot}.urdf.xacro')
	if not os.path.isfile(xacro_path):
		available = sorted(d for d in os.listdir(os.path.join(desc_share, 'arms'))
		                   if os.path.isfile(os.path.join(desc_share, 'arms', d, 'urdf', f'{d}.urdf.xacro')))
		raise RuntimeError(f'未知机型 {robot!r}: 找不到 {xacro_path}. 可用机型: {available}')

	# 转发本链声明的参数 (链不认识的参数不传, 各链默认值生效)
	launch_args = {}
	for name in arg_names:
		if name == 'robot':
			launch_args['robot'] = robot
			continue
		# 只转发用户显式给过的值 (默认值由子链自己定, 双层默认不漂移)
		cfg = LaunchConfiguration(name)
		if context.launch_configurations.get(name) is not None:
			launch_args[name] = cfg.perform(context)

	return [IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory(pkg), 'launch', launch_file)),
		launch_arguments=launch_args.items(),
	)]


def generate_launch_description():
	return LaunchDescription([
		DeclareLaunchArgument('chain', default_value='mock',
		                      description='仿真链: mock(kinematic) | gz(Gazebo Fortress) | mujoco(MuJoCo)'),
		DeclareLaunchArgument('robot',
		                      description='机型名(必填): piper | xarm7'),
		DeclareLaunchArgument('use_rviz', default_value='false',
		                      description='是否启动 RViz2 (三链通用)'),
		DeclareLaunchArgument('gui', default_value='true',
		                      description='[仅 gz 链] Gazebo GUI 客户端'),
		DeclareLaunchArgument('headless', default_value='true',
		                      description='[仅 mujoco 链] 无头模式(默认开); false 拉渲染窗'),
		OpaqueFunction(function=_launch_setup),
	])
