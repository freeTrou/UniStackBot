"""MoveIt2 接入 launch (M1, piper 试点)。

用法: 先起链, 再起本 launch (第二终端):
  ros2 launch unistackbot_moveit_config moveit_piper.launch.py
本 launch 不自带 ros2_control_node/controller_manager —— 它连接既有链
(control.launch.py), 只补 move_group + RViz。链上 JTC 需已注册 (inactive),
执行窗口切换由上层脚本/编排负责 (见 scripts/demo_moveit_plan.py)。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import yaml
import os
import ament_index_python


def load_yaml(pkg, rel):
    path = os.path.join(ament_index_python.get_package_share_directory(pkg), rel)
    with open(path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    robot_arg = DeclareLaunchArgument('robot', default_value='piper',
                                      description='机型 (M1 仅 piper 有 SRDF/MoveIt 配置)')

    # M1 范围守卫: 非本机型无 SRDF/规划组配置, fail-fast 免得隐性错配
    # (xacro 会按 <robot> 展开, 但 SRDF 是 piper 的 → 规划组对不上)
    from launch.actions import OpaqueFunction

    def _guard(context):
        robot = LaunchConfiguration('robot').perform(context)
        if robot != 'piper':
            raise RuntimeError(
                "M1 仅支持 piper (收到 '%s'); 其他机型的 MoveIt 配置属 M2+ "
                "-- 见 unistackbot_moveit_config/README" % robot)
        return []
    guard = OpaqueFunction(function=_guard)

    # robot_description: 与链同源 xacro (use_world:=true, 同 control.launch.py 口径);
    # URDF/SRDF 字符串参数须包 ParameterValue(str) —— 直接传会被当 yaml 解析炸掉
    # (gz 链同款坑, CLAUDE.md 有案)
    robot_description = {
        'robot_description': ParameterValue(Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([FindPackageShare('unistackbot_description'),
                                  'arms', LaunchConfiguration('robot'),
                                  'urdf', 'piper.urdf.xacro']),
            ' use_world:=true use_ros2_control:=true']), value_type=str)
    }
    cfg_dir = PathJoinSubstitution([FindPackageShare('unistackbot_moveit_config'), 'config'])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(Command([
            FindExecutable(name='cat'), ' ',
            PathJoinSubstitution([cfg_dir, 'piper.srdf'])]), value_type=str)
    }

    move_group_params = [
        robot_description,
        robot_description_semantic,
        {'robot_description_kinematics': load_yaml('unistackbot_moveit_config',
                                                   'config/piper_kinematics.yaml')},
        {'robot_description_planning': load_yaml('unistackbot_moveit_config',
                                                 'config/piper_joint_limits.yaml')},
        {'planning_pipelines': ['ompl'],
         'default_planning_pipeline': 'ompl',
         'ompl': load_yaml('unistackbot_moveit_config', 'config/piper_ompl_planning.yaml')},
        load_yaml('unistackbot_moveit_config', 'config/piper_moveit_controllers.yaml'),
        {'moveit_manage_controllers': True},
        {'allow_trajectory_execution': True},
        {'publish_planning_scene': True},
        {'publish_geometry_updates': True},
        {'publish_state_updates': True},
        {'publish_transforms_updates': True},
        {'publish_robot_description': True},
        {'publish_robot_description_semantic': True},
        # planning_scene_monitor 话题保持默认 (/monitored_planning_scene) ——
        # 覆盖成 /moveit_cpp/* 会与 RViz MotionPlanning 插件订阅错位 (display
        # 永远等不到场景, 2026-10-09 实锤)
    ]

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group',
        output='screen', parameters=move_group_params)

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution([cfg_dir, 'piper_moveit.rviz'])],
        parameters=[robot_description, robot_description_semantic,
                    {'robot_description_kinematics': load_yaml(
                        'unistackbot_moveit_config', 'config/piper_kinematics.yaml')}])

    return LaunchDescription([robot_arg, move_group, rviz])
