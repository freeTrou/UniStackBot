import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    robot = LaunchConfiguration('robot').perform(context)
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'

    desc_share = get_package_share_directory('unistackbot_description')
    xacro_path = os.path.join(desc_share, 'arms', robot, 'urdf', f'{robot}.urdf.xacro')

    # 显式错误流: 机型不存在时报错并列出可用项, 不让 launch 深处崩一个难懂的面板
    arms_root = os.path.join(desc_share, 'arms')
    if not os.path.isfile(xacro_path):
        available = sorted(d for d in os.listdir(arms_root)
                           if os.path.isfile(os.path.join(arms_root, d, 'urdf', f'{d}.urdf.xacro')))
        raise RuntimeError(f'未知机型 {robot!r}: 找不到 {xacro_path}. 可用机型: {available}')

    # MJCF 资产 fail-fast (mujoco 链独有: 无 MJCF 的机型不可起链)
    mjcf_path = os.path.join(desc_share, 'arms', robot, 'mujoco', f'{robot}.xml')
    if not os.path.isfile(mjcf_path):
        available = sorted(d for d in os.listdir(arms_root)
                           if os.path.isfile(os.path.join(arms_root, d, 'mujoco', f'{d}.xml')))
        raise RuntimeError(f'机型 {robot!r} 缺 MJCF 资产: 找不到 {mjcf_path}. '
                           f'已有 MJCF 的机型: {available or "无"} (生成方法见 arms/{robot}/mujoco/README.md)')

    # 渲染 URDF (world 固定基座; mujoco 分支 → MujocoSystemInterface, 手指 passive)
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_path,
        ' use_ros2_control:=true',
        ' use_world:=true',
        ' use_gazebo:=mujoco',
        f' headless:={"true" if headless else "false"}',
    ]).perform(context)

    # 压成单行: 定制 ros2_control_node 沿用 rcl 参数解析器, 值中含换行会被拒
    # (与 gz 链同一坑; robot_description 参数直供, 不走 demo 的话题+remap 路线)
    robot_description_content = ET.tostring(
        ET.fromstring(robot_description_content), encoding='unicode')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': ParameterValue(robot_description_content, value_type=str)},
            {'use_sim_time': True},
        ],
        output='screen',
    )

    controllers_yaml = os.path.join(
        get_package_share_directory('unistackbot_bringup'),
        'config', f'{robot}_controllers.yaml',
    )
    if not os.path.isfile(controllers_yaml):
        available = sorted(f for f in os.listdir(os.path.dirname(controllers_yaml))
                           if f.endswith('_controllers.yaml'))
        raise RuntimeError(f'机型 {robot!r} 缺控制器配置: 找不到 {controllers_yaml}. 现有配置: {available}')

    # mujoco_ros2_control 的定制 ros2_control_node (等上游 ros2_control 合入仿真
    # PR 后可换回标准节点——临时措施): controller_manager + MuJoCo 引擎 + 物理线程
    # 同进程; RT 线程调优 (thread_priority/cpu_affinity) 走 controllers yaml 官方参数,
    # 与 mock 链同一份配置。双线程架构 (CM RT 线程 + MuJoCo 物理线程) 保持。
    control_node_kwargs = {}
    if headless:
        # apt 0.1.2 无 MUJOCO_HEADLESS 环境变量支持 (PR #157 未随发布), headless 走
        # URDF 硬件参数; 此处 env 为后续版本升级的保险, 无害
        control_node_kwargs['additional_env'] = {'MUJOCO_HEADLESS': '1'}
    mujoco_control = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': ParameterValue(robot_description_content, value_type=str)},
            controllers_yaml,
        ],
        output='screen',
        **control_node_kwargs,
    )

    # /sim_control 的 mujoco 适配器 (0d, 2026-09-21): 桥原生四服务; 诚实能力矩阵见其源码头
    sim_control_mujoco = Node(
        package='unistackbot_mujoco',
        executable='sim_control_mujoco_node',
        output='screen',
    )

    nodes = [robot_state_publisher, mujoco_control, sim_control_mujoco]

    if use_rviz:
        rviz_config = PathJoinSubstitution([
            FindPackageShare('unistackbot_description'),
            'config', 'rviz.rviz',
        ])
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ))

    spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_stream_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
        Node(
        	package='controller_manager',
        	executable='spawner',
        	arguments=['ee_state_broadcaster', '--controller-manager', '/controller_manager'],
        	output='screen',
        ),
        Node(
        	package='unistackbot_bringup',
        	executable='supervisor_node',
        	output='screen',
        ),
        # 笛卡尔流式控制器: 以 inactive 注册 (接口独占, 与 JointStream 由 switch_controllers 切换)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['cartesian_motion_controller', '--controller-manager',
                       '/controller_manager', '--inactive'],
            output='screen',
        ),
    ]

    return nodes + spawners


def generate_launch_description():
    return LaunchDescription([
        # DDS 跟随机器默认配置 (~/cyclonedds.xml), launch 不再覆盖
        DeclareLaunchArgument('robot',
                              description='机型名(必填), 对应 unistackbot_description/arms/<robot>/ 与 bringup config/<robot>_controllers.yaml'),
        DeclareLaunchArgument('headless', default_value='true',
                              description='无头模式 (不拉 MuJoCo Simulate 渲染窗); false 时需可用 DISPLAY'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='是否启动 RViz2'),
        OpaqueFunction(function=_launch_setup),
    ])
