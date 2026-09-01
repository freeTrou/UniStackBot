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
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    gui = LaunchConfiguration('gui').perform(context) == 'true'

    xacro_path = PathJoinSubstitution([
        FindPackageShare('unistackbot_description'),
        'arms', 'piper', 'urdf', 'piper.urdf.xacro',
    ])
    # 渲染 URDF
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_path,
        ' use_gripper:=true',
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
    urdf_path = os.path.join(tempfile.gettempdir(), 'unistackbot_piper_gz.urdf')
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
            '-name', 'piper',
            '-x', '0', '-y', '0', '-z', '0',
        ],
        output='screen',
    )

    # /clock 桥接: controller_manager 使用仿真时间
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # 统一仿真控制层 gz 后端适配器: /sim_control/pause|resume|step -> ign 世界服务
    sim_control_gz = Node(
        package='unistackbot_sim_control',
        executable='sim_control_gz_node',
        parameters=[{'world': 'piper_world'}],
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

    # 资源路径放首位: 先设环境再拉起后续进程
    return [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resource_path),
        robot_state_publisher,
        gz_sim,
        clock_bridge,
        sim_control_gz,
        spawn_entity,
    ] + spawners

# 入口
def generate_launch_description():
    return LaunchDescription([
        # DDS 跟随机器默认配置 (~/cyclonedds.xml, 本机统一配置 lo + 单播 peer),
        # launch 不再覆盖; 见 CLAUDE.md 的 DDS 说明
        # 声明必须在 OpaqueFunction 之前: _launch_setup 会 perform 这些配置
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo GUI 客户端'),
        OpaqueFunction(function=_launch_setup), # 启动py函数 
    ])
