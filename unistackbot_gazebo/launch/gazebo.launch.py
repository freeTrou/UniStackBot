import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
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

    xacro_path = PathJoinSubstitution([
        FindPackageShare('unistackbot_description'),
        'arms', 'piper', 'urdf', 'piper.urdf.xacro',
    ])
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_path,
        ' use_gripper:=true',
        ' use_ros2_control:=true',
        ' use_world:=false',
        ' use_gazebo:=true',
    ])

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
            '-topic', 'robot_description',
            '-entity', 'piper',
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
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo GUI 客户端'),
        OpaqueFunction(function=_launch_setup),
    ])
