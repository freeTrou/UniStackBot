import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'
    robot = LaunchConfiguration('robot').perform(context)

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
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        # 同上: 裸字符串会被 launch_ros 当 YAML 解析, 必须 ParameterValue(str)
        parameters=[
            {'robot_description': ParameterValue(robot_description_content, value_type=str)},
            controllers_yaml,
        ],
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

    return nodes + spawners


def generate_launch_description():
    return LaunchDescription([
        # DDS 跟随机器默认配置 (~/cyclonedds.xml), launch 不再覆盖
        DeclareLaunchArgument('robot',
                              description='机型名(必填), 对应 unistackbot_description/arms/<robot>/ 与 bringup config/<robot>_controllers.yaml'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='是否启动 RViz2'),
        OpaqueFunction(function=_launch_setup),
    ])
