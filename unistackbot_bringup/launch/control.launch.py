import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'
    robot = LaunchConfiguration('robot').perform(context)
    bus_hz = LaunchConfiguration('bus_hz').perform(context)

    desc_share = get_package_share_directory('unistackbot_description')
    xacro_path = os.path.join(desc_share, 'arms', robot, 'urdf', f'{robot}.urdf.xacro')

    # 显式错误流: 机型不存在时报错并列出可用项, 不让 launch 深处崩一个难懂的面板
    arms_root = os.path.join(desc_share, 'arms')
    if not os.path.isfile(xacro_path):
        available = sorted(d for d in os.listdir(arms_root)
                           if os.path.isfile(os.path.join(arms_root, d, 'urdf', f'{d}.urdf.xacro')))
        raise RuntimeError(f'未知机型 {robot!r}: 找不到 {xacro_path}. 可用机型: {available}')

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_path,
        ' use_ros2_control:=true',
        ' use_world:=true',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # ParameterValue(str) 必须: launch_ros 会把裸字符串当 YAML 解析,
        # 多行 URDF 会直接让 launch 报错
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str)}],
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
    # bus_hz (频率矩阵测试, 2026-09-23): 运行期覆盖 update_rate (yaml /** 通配节的
    # 单一事实源被 launch 的 -p 全局覆盖压过) —— CM RT 循环与全部控制器节点同拍切换。
    # 空 = 不覆盖, yaml 值生效。gz 链 CM 在 gz 插件进程内, 此机制不可达 (ign launch 会拒)。
    cm_parameters = [
        {'robot_description': ParameterValue(robot_description_content, value_type=str)},
        controllers_yaml,
    ]
    if bus_hz:
        cm_parameters.append({'update_rate': int(bus_hz)})
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        # 同上: 裸字符串会被 launch_ros 当 YAML 解析, 必须 ParameterValue(str)
        parameters=cm_parameters,
        output='screen',
    )

    nodes = [robot_state_publisher, ros2_control_node]

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

    # spawner 竞态防御 (2026-09-23, 官方依据 ros2_control issue #2071):
    #   根因不是"没重试"(spawner 内建 3 次, controller_manager_services.py max_attempts=3),
    #   而是 10s service-call 窗口内 CM 忙(硬件 on_init 占 executor)响应未归 → 盲目重试
    #   撞进首个请求已执行的半途状态 ("already loaded"+"no controller with this name")。
    #   官方维护者两方向: ①delay the spawners(TimerAction 2s) ②launch_utils/example_13
    #   模式 = 一个 spawner 进程传控制器列表, 收拢并发服务调用风暴。--service-call-timeout
    #   30s 让 3 次内建重试变耐心(实测疲劳机硬件 init >4s)。--controller-manager-timeout
    #   默认 0.0 已是永远等服务, 不调。RMW 层 wait 卡死无解, 由流水线 ⓪ 残留检查兜底。
    controller_spawners = [
        # 激活组: 一个 spawner 进程按序 load/configure/activate (官方 example_13 模式)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', 'joint_stream_controller',
                       'ee_state_broadcaster',
                       '--controller-manager', '/controller_manager',
                       '--service-call-timeout', '30'],
            output='screen',
        ),
        # 笛卡尔流式控制器: 以 inactive 注册 (接口独占, switch_controllers 切换)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['cartesian_motion_controller',
                       '--controller-manager', '/controller_manager',
                       '--service-call-timeout', '30', '--inactive'],
            output='screen',
        ),
    ]

    return nodes + [
        Node(
        	package='unistackbot_bringup',
        	executable='supervisor_node',
        	output='screen',
        ),
        TimerAction(period=2.0, actions=controller_spawners),
    ]


def generate_launch_description():
    return LaunchDescription([
        # DDS 跟随机器默认配置 (~/cyclonedds.xml), launch 不再覆盖
        DeclareLaunchArgument('robot',
                              description='机型名(必填), 对应 unistackbot_description/arms/<robot>/ 与 bringup config/<robot>_controllers.yaml'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='是否启动 RViz2'),
        DeclareLaunchArgument('bus_hz', default_value='',
                              description='总线/CM 频率覆盖 (Hz, 如 500/1000); 空=yaml update_rate 生效'),
        OpaqueFunction(function=_launch_setup),
    ])
