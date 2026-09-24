import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    model = LaunchConfiguration('model').perform(context)
    if not model:
        # 机型无默认值 (2026-09-22 纪律): 必须显式指定, fail-fast 列可用项
        import os as _os
        try:
            from ament_index_python.packages import get_package_share_directory
            _arms = _os.path.join(
                get_package_share_directory('unistackbot_description'), 'arms')
            _avail = ', '.join(sorted(_os.listdir(_arms)))
        except Exception:
            _avail = '未知'
        raise RuntimeError(
            '缺必填参数 model (机型 xacro/urdf 路径)。可用机型: ' + _avail)
    use_gripper = LaunchConfiguration('use_gripper').perform(context)
    use_ros2_control = LaunchConfiguration('use_ros2_control').perform(context)
    use_world = LaunchConfiguration('use_world').perform(context)
    use_gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('rviz')

    is_xacro = model.endswith('.xacro') or model.endswith('.urdf.xacro')
    if is_xacro:
        robot_description_content = Command([
            FindExecutable(name='xacro'), ' ', model,
            ' use_gripper:=', use_gripper,
            ' use_ros2_control:=', use_ros2_control,
            ' use_world:=', use_world,
        ])
    else:
        robot_description_content = Command([
            FindExecutable(name='cat'), ' ', model
        ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # ParameterValue(str) 必须: launch_ros 会把裸字符串当 YAML 解析,
        # 多行 URDF 会直接让 launch 报错
        parameters=[{'robot_description': ParameterValue(robot_description_content, value_type=str)}],
        output='screen'
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(use_gui)
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(use_gui)
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('unistackbot_description'),
        'config', 'rviz.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return [
        robot_state_publisher,
        joint_state_publisher_gui,
        joint_state_publisher,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            description='URDF/Xacro 文件路径（必填, 机型无默认值; '
                        '例: $(ros2 pkg prefix --share unistackbot_description)/arms/xarm7/urdf/xarm7.urdf.xacro）'
        ),
        DeclareLaunchArgument(
            'use_gripper', default_value='true',
            description='是否加载夹爪'
        ),
        DeclareLaunchArgument(
            'use_ros2_control', default_value='false',
            description='是否加载 ros2_control 标签'
        ),
        DeclareLaunchArgument(
            'use_world', default_value='true',
            description='是否添加 world 固定根坐标系'
        ),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='是否启动 joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='是否启动 RViz2'
        ),
        OpaqueFunction(function=_launch_setup),
    ])
