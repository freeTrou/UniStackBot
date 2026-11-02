"""产品级一键整机 launch (2026-11-02): 控制链 + /sdk 网关 + 可选 FakeBoard。

对外 = 一个整机一条命令; 对内保持进程隔离 (门面崩不伤控制链)。

用法:
  ros2 launch mos_compat sdk_machine.launch.py robot:=mos_p20
  ros2 launch mos_compat sdk_machine.launch.py robot:=mos_p20 backend:=udp_board fake_board:=true

args:
  robot       (必填) 机型名 (bringup 机型发现: 框架类别目录)
  backend     kinematic (mock 默认) | udp_board
  board_ip / board_port / bind_port   udp_board 部署值 (透传 bringup)
  fake_board  true 时拉起 fake_board_daemon (仅 udp_board 有意义; 真机部署不拉)
  js_controllers  四件式 JS 实例表 (默认 mos_p20 编组)
  use_rviz    透传 bringup
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('unistackbot_bringup')
    robot = LaunchConfiguration('robot')

    # 网关参数文件 = 随机型控制器 yaml (内含 mos_sdk_interface 段; 路径含机型名须 OpaqueFunction 内解析)
    from launch.actions import OpaqueFunction

    def _gateway_node(context):
        robot_name = LaunchConfiguration('robot').perform(context)
        yaml_path = os.path.join(bringup_share, 'config', f'{robot_name}_controllers.yaml')
        if not os.path.isfile(yaml_path):
            raise RuntimeError(f'缺机型配置 (含 mos_sdk_interface 段): {yaml_path}')
        return [
            Node(
                package='mos_compat',
                executable='mos_compat_node',
                output='screen',
                arguments=['--ros-args', '--params-file', yaml_path],
            ),
            # WB inactive 常驻装载 (产品 launch 职责: 框架 control.launch 保持通用不带 WB;
            # 6s 让路 = bringup spawner 竞态防御 TimerAction(2s)+装载时宽 同款口径)
            TimerAction(period=6.0, actions=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=['whole_body_controller', '--controller-manager', '/controller_manager',
                           '--service-call-timeout', '30', '--inactive'],
                output='screen',
            )]),
        ]

    chain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'control.launch.py')),
        launch_arguments={
            'robot': robot,
            'backend': LaunchConfiguration('backend'),
            'board_ip': LaunchConfiguration('board_ip'),
            'board_port': LaunchConfiguration('board_port'),
            'bind_port': LaunchConfiguration('bind_port'),
            'js_controllers': LaunchConfiguration('js_controllers'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )

    fake_board = Node(
        package='unistackbot_udp_board',
        executable='fake_board_daemon',
        output='screen',
        condition=IfCondition(LaunchConfiguration('fake_board')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot', description='机型名(必填), bringup 机型发现'),
        DeclareLaunchArgument('backend', default_value='kinematic',
                              description='kinematic (mock 默认) | udp_board'),
        DeclareLaunchArgument('board_ip', default_value='127.0.0.1'),
        DeclareLaunchArgument('board_port', default_value='10086'),
        DeclareLaunchArgument('bind_port', default_value='9090'),
        DeclareLaunchArgument('fake_board', default_value='false',
                              description='true = 拉起 FakeBoard (udp_board 联调用)'),
        DeclareLaunchArgument('js_controllers', default_value='[js_arms, js_head, js_torso, js_gripper]',
                              description='JS 实例名列表 (随机型编组)'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        chain,
        fake_board,
        OpaqueFunction(function=_gateway_node),
    ])
