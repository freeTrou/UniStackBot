#!/usr/bin/env python3
"""给 Piper 的 JTC 发送一段往返演示轨迹 (抬臂 -> 转向 + 夹爪开合 -> 回零).

前置: piper_ign.launch.py / gazebo.launch.py / piper_control.launch.py 任一已启动,
且 unistackbot_bringup 的控制器配置里 joint_trajectory_controller 处于 active。

用法: ros2 run unistackbot_bringup piper_demo_motion.py
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# 与 piper_controllers.yaml 中 JTC 的关节列表一致 (须全量, JTC 默认不接受子集)
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']

# (各关节目标弧度, 该段耗时秒) — 限位见 piper_ros2_control.xacro 的 min/max
WAYPOINTS = [
    ([0.8, 0.6, -1.0, 0.0, 0.0, 0.0, 0.05], 3.0),
    ([-0.9, 1.2, -1.6, 0.4, -0.4, 0.3, 0.09], 6.0),
    ([0.0, 0.3, -0.3, 0.0, 0.0, 0.0, 0.02], 9.0),
    ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 12.0),
]


class PiperDemoMotion(Node):

    def __init__(self):
        super().__init__('piper_demo_motion')
        self._client = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')

    def run(self):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                '10 秒内未找到 joint_trajectory_controller 的 action 服务器, 可能原因:\n'
                '  1) 仿真未启动: ros2 launch unistackbot_gazebo piper_ign.launch.py\n'
                '  2) 控制器未激活: 等 launch 日志出现两行 "Configured and activated"\n'
                '  3) 有残留进程冲突: ros2 run unistackbot_gazebo gz_clean.sh')
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for positions, t in WAYPOINTS:
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in positions]
            point.time_from_start.sec = int(t)
            goal.trajectory.points.append(point)

        self.get_logger().info('发送轨迹 (共 %d 个路径点, 约 %d 秒)...' % (
            len(WAYPOINTS), WAYPOINTS[-1][1]))
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被控制器拒绝')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        self.get_logger().info('轨迹执行结束, 状态码 %d (4=成功)' % status)


def main(args=None):
    rclpy.init(args=args)
    node = PiperDemoMotion()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
