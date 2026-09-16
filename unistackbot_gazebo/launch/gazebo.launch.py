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
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    robot = LaunchConfiguration('robot').perform(context)

    desc_share = get_package_share_directory('unistackbot_description')
    xacro_path = os.path.join(desc_share, 'arms', robot, 'urdf', f'{robot}.urdf.xacro')

    # 显式错误流: 机型不存在时报错并列出可用项
    arms_root = os.path.join(desc_share, 'arms')
    if not os.path.isfile(xacro_path):
        available = sorted(d for d in os.listdir(arms_root)
                           if os.path.isfile(os.path.join(arms_root, d, 'urdf', f'{d}.urdf.xacro')))
        raise RuntimeError(f'未知机型 {robot!r}: 找不到 {xacro_path}. 可用机型: {available}')

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_path,
        ' use_ros2_control:=true',
        # world 链接把基座固定在世界原点: 否则基座自由浮动, 机械臂会在重力下瘫倒
        ' use_world:=true',
        ' use_gazebo:=true',
    ]).perform(context)

    # gazebo_ros2_control 会把 URDF 以 `--param robot_description:=<urdf>` 规则
    # 转发给 controller_manager 节点, 而 rcl 参数解析器不允许值中含换行,
    # 多行 xacro 输出会让 CM 创建失败 (spawner 永远等不到 /controller_manager),
    # 这里重新序列化压成单行。
    robot_description_content = ET.tostring(
        ET.fromstring(robot_description_content), encoding='unicode')

    # spawn_entity 用 -file 直接读 URDF, 不走 /robot_description 话题:
    # TRANSIENT_LOCAL 迟到补发在 iceoryx 共享内存等 DDS 配置下不可靠,
    # 走话题会让 spawn_entity 永远等不到模型描述。
    urdf_path = os.path.join(tempfile.gettempdir(), f'unistackbot_{robot}_gazebo.urdf')
    with open(urdf_path, 'w') as f:
        f.write(robot_description_content)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen',
    )

    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    world_path = os.path.join(
        get_package_share_directory('unistackbot_gazebo'),
        'worlds', 'empty.world',
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')
        ),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', urdf_path,
            '-entity', robot,
            '-x', '0', '-y', '0', '-z', '0',
        ],
        output='screen',
    )

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
            arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        ),
    ]

    nodes = [robot_state_publisher, gzserver, spawn_entity] + spawners
    if gui:
        nodes.append(gzclient)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        # Gazebo 自动探测的公示地址可能指向不可达的虚拟网卡 (如 10.0.3.x 网桥),
        # gzclient 会连黑洞导致界面冻结, 单机仿真固定公示 127.0.0.1
        SetEnvironmentVariable('GAZEBO_IP', '127.0.0.1'),
        # 跳过 models.gazebosim.org 在线模型库访问, 避免离线环境下启动阻塞
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        # DDS 跟随机器默认配置 (~/cyclonedds.xml), launch 不再覆盖
        DeclareLaunchArgument('robot',
                              description='机型名(必填), 对应 unistackbot_description/arms/<robot>/'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo GUI 客户端'),
        OpaqueFunction(function=_launch_setup),
    ])
