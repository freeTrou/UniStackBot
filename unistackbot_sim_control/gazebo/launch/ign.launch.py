import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'
    robot = LaunchConfiguration('robot').perform(context)
    # bus_hz (mock/mujoco 链的频率矩阵参数) 在 gz 链**忽略不报错** (用户裁决 2026-09-23:
    # 三链各有各端功能, gz 不走换频) —— CM 住在 gz_ros2_control 插件进程内, launch 级
    # -p 覆盖本就不可达; gz 真要换频改 yaml /**.update_rate
    _ = LaunchConfiguration('bus_hz').perform(context)

    desc_share = get_package_share_directory('unistackbot_description')
    # 形态分类目录 (2026-10-14 资产分类: arms/mobile_manipulators/humanoids);
    # 机型发现 = 各类别下 glob, 无 robot→类别映射 (映射表 = 第二份要维护的硬编码)
    CATEGORIES = ('arms', 'mobile_manipulators', 'humanoids')
    def find_robot_dir(desc_share, robot):
        for cat in CATEGORIES:
            d = os.path.join(desc_share, cat, robot)
            if os.path.isdir(d):
                return d
        return None

    robot_dir = find_robot_dir(desc_share, robot)
    if robot_dir is None:
        available = sorted(r for cat in CATEGORIES for r in os.listdir(os.path.join(desc_share, cat))
                           if os.path.isfile(os.path.join(desc_share, cat, r, 'urdf', f'{r}.urdf.xacro')))
        raise RuntimeError(f'未知机型 {robot!r}: 各类别目录下均无。可用机型: {available}')
    xacro_path = os.path.join(robot_dir, 'urdf', f'{robot}.urdf.xacro')

    # 显式错误流: 机型不存在时报错并列出可用项
    # 渲染 URDF
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_path,
        ' use_ros2_control:=true',
        # world 链接把基座固定在世界原点: 否则基座自由浮动, 机械臂会在重力下瘫倒
        ' use_world:=true',
        ' use_gazebo:=ign',
    ]).perform(context)

    # 压成单行: gz_ros2_control 同样把 URDF 以 --param 规则转发给 controller_manager,
    # rcl 参数解析器不接受值中含换行
    robot_description_content = ET.tostring(
        ET.fromstring(robot_description_content), encoding='unicode')

    # create 用 -file 直接读 URDF, 不走 /robot_description 话题
    urdf_path = os.path.join(tempfile.gettempdir(), f'unistackbot_{robot}_gz.urdf')
    with open(urdf_path, 'w') as f:
        f.write(robot_description_content)

    # Fortress 靠 IGN_GAZEBO_RESOURCE_PATH 解析 model:// 资源: URDF->SDF 转换会把
    # package:// 重写为 model://, 必须把包含包目录的 ament share 根目录加进去,
    # 否则 GUI/服务端都找不到 mesh (Classic 由 gazebo_ros 自动完成, ign 没有)
    ign_resource_root = os.path.dirname(
        get_package_share_directory('unistackbot_description'))
    ign_resource_path = os.pathsep.join(
        p for p in [ign_resource_root, os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')] if p)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # ParameterValue(str) 必须: launch_ros 会把裸字符串当 YAML 解析
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str)}],
        output='screen',
    )

    world_path = os.path.join(
        get_package_share_directory('unistackbot_gazebo'), 'worlds', 'empty_ign.world')

    # -r 启动即运行; gui:=false 时 -s 只起服务端
    gz_args = f'-r {"-s " if not gui else ""}{world_path}'
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py',
            ])
        ),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_path,
            '-name', robot,
            '-x', '0', '-y', '0', '-z', '0',
        ],
        output='screen',
    )

    # 共享 world 名: launch 内单一事实源 (bridge 服务串与适配器参数同源),
    # 仍须与 empty_ign.world 的 <world name> 一致 (SDF 侧)
    world_name = 'unistack_world'

    # 桥接: /clock 供 CM 仿真时间; /stats 供 RTF/暂停态监控; 世界控制服务供
    # sim_control_gz 适配器调 pause/resume/step
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/stats@ros_gz_interfaces/msg/WorldStatistics[gz.msgs.WorldStatistics',
            f'/world/{world_name}/control@ros_gz_interfaces/srv/ControlWorld',
        ],
        output='screen',
    )

    # 统一仿真控制层的 gz 后端适配器 (/sim_control 契约的 gz 实现, 编译在本包;
    # 契约住 sim_control 包). world 参数与 empty_ign.world 的 <world name> 及桥接服务名一致
    sim_control_gz = Node(
        package='unistackbot_gazebo',
        executable='sim_control_gz_node',
        parameters=[{'world': world_name}],
        output='screen',
    )

    # spawner 竞态防御 (2026-09-23, 官方依据 ros2_control issue #2071):
    #   根因不是"没重试"(spawner 内建 3 次, max_attempts=3), 而是 10s service-call 窗口内
    #   CM 忙(硬件 on_init 占 executor)响应未归 → 盲目重试撞半途状态。官方两方向:
    #   ①delay the spawners(TimerAction 2s) ②launch_utils/example_13 模式 = 一个 spawner
    #   进程传控制器列表收拢并发风暴。--service-call-timeout 30s 让内建重试变耐心。
    #   RMW 层 wait 卡死无解, 由流水线 ⓪ 残留检查兜底; supervisor 即时拉起(纯订阅容忍迟到)。
    # JS 实例按类型发现 (2026-10-14 四件式, 与 mujoco 链同口径): yaml controller_manager
    # 节里 type 含 JointStreamController 的条目即实例名 (单块机型=joint_stream_controller,
    # 多实例机型 r1_pro=js_arms+js_torso); gz 链 yaml 由 URDF <gazebo> 块消费, 此处路径同源
    import yaml
    gz_controllers_yaml = os.path.join(
        get_package_share_directory('unistackbot_bringup'),
        'config', f'{robot}_controllers.yaml')
    with open(gz_controllers_yaml) as f:
        cm_params = yaml.safe_load(f)['controller_manager']['ros__parameters']
    js_instances = sorted(k for k, v in cm_params.items()
                          if isinstance(v, dict)
                          and 'JointStreamController' in str(v.get('type', '')))
    if not js_instances:
        raise RuntimeError(f'{gz_controllers_yaml} 未声明任何 JointStreamController 实例')
    # 底盘控制器 (2026-10-15 底盘域 B2): yaml 声明才进激活组 (机型无关发现, 同 JS)
    base_instances = sorted(k for k, v in cm_params.items()
                            if isinstance(v, dict)
                            and 'BaseVelocityController' in str(v.get('type', '')))

    controller_spawners = [
        # 激活组: 一个 spawner 进程按序 load/configure/activate (官方 example_13 模式)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'] + js_instances + base_instances
                      + ['ee_state_broadcaster',
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

    # 资源路径放首位: 先设环境再拉起后续进程
    nodes = [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path),
        robot_state_publisher,
        gz_sim,
        gz_bridge,
        sim_control_gz,
        spawn_entity,
        Node(
        	package='unistackbot_bringup',
        	executable='supervisor_node',
        	output='screen',
        ),
        TimerAction(period=2.0, actions=controller_spawners),
    ]

    # RViz 可选 (与 mock 链 control.launch.py 对齐; use_world:=true 时固定系为 world)
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
    return nodes

# 入口
def generate_launch_description():
    return LaunchDescription([
        # DDS 跟随机器默认配置 (~/cyclonedds.xml, 本机统一配置 lo + 单播 peer),
        # launch 不再覆盖; 见 CLAUDE.md 的 DDS 说明
        # 声明必须在 OpaqueFunction 之前: _launch_setup 会 perform 这些配置
        DeclareLaunchArgument('robot',
                              description='机型名(必填), 对应 unistackbot_description/arms/<robot>/'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo GUI 客户端'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='是否启动 RViz2 (gui:=false 无头模式下也可用)'),
        DeclareLaunchArgument('bus_hz', default_value='',
                              description='gz 链忽略此参数 (mock/mujoco 专用; 三链各有各端功能)'),
        OpaqueFunction(function=_launch_setup), # 启动py函数
    ])
