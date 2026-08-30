import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    use_rviz = LaunchConfiguration('use_rviz').perform(context) == 'true'

    xacro_path = PathJoinSubstitution([
        FindPackageShare('unistackbot_description'),
        'arms', 'piper', 'urdf', 'piper.urdf.xacro',
    ])
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_path,
        ' use_gripper:=true',
        ' use_ros2_control:=true',
        ' use_world:=true',
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen',
    )

    controllers_yaml = os.path.join(
        get_package_share_directory('unistackbot_bringup'),
        'config', 'piper_controllers.yaml',
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_content}, controllers_yaml],
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
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='是否启动 RViz2'),
        OpaqueFunction(function=_launch_setup),
    ])
