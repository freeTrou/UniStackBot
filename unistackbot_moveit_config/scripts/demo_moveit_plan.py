#!/usr/bin/env python3
"""MoveIt 规划→JTC 执行 E2E (M1, 程序化; RViz 交互留给用户手测)。

链路: /plan_kinematic_path 服务 (joint-space 目标) → 取轨迹 →
  切换 JS→JTC (执行窗口) → FollowJointTrajectory action 执行 → 切回 JS。
两域断流语义裁决 (2026-10-08): 任务域中止/断流 = JTC 保持最后点 (主流做法);
编排失败的人话恢复指令照 demo_cartesian finally 纪律。

前置: control.launch.py 已起链 (JTC inactive 已注册), moveit_piper.launch.py 已起
move_group。用法: ros2 run unistackbot_moveit_config demo_moveit_plan.py
        [--j2 0.6] [--plan-timeout 15]
"""

import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from moveit_msgs.srv import GetMotionPlan

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
ARM = {  # home 起点 (SRDF 同款, 限位内缩)
    'joint1': 0.0, 'joint2': 0.15, 'joint3': -0.15,
    'joint4': 0.0, 'joint5': 0.0, 'joint6': 0.0,
}


def switch(*args):
    r = subprocess.run(['ros2', 'control', 'switch_controllers', *args],
                       capture_output=True, text=True, timeout=15)
    if r.returncode != 0:
        raise RuntimeError('switch_controllers 失败: ' + r.stderr.strip()[-160:])


class MovePlanClient(Node):

    def __init__(self):
        super().__init__('demo_moveit_plan')
        self.cli = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.action = ActionClient(self, FollowJointTrajectory,
                                   '/joint_trajectory_controller/follow_joint_trajectory')

    def plan(self, j2_target, timeout):
        if not self.cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/plan_kinematic_path 10s 未出现 (move_group 未起?)')
        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = 'arm'
        mpr.num_planning_attempts = 5
        mpr.allowed_planning_time = 5.0
        mpr.start_state.joint_state.name = JOINTS
        mpr.start_state.joint_state.position = [ARM[j] for j in JOINTS]
        goal = Constraints()
        goal.joint_constraints = [
            JointConstraint(joint_name=j, position=(j2_target if j == 'joint2'
                                                    else ARM[j]),
                            tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
            for j in JOINTS]
        mpr.goal_constraints = [goal]
        req.motion_plan_request = mpr
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None:
            raise RuntimeError('规划请求 %.0fs 无响应' % timeout)
        resp = fut.result()
        err = resp.motion_plan_response.error_code.val
        if err != 1:
            raise RuntimeError('规划失败 error_code=%d (1=SUCCESS)' % err)
        traj = resp.motion_plan_response.trajectory.joint_trajectory
        self.get_logger().info('规划成功: %d 路点, %.2fs' % (
            len(traj.points), traj.points[-1].time_from_start.sec +
            traj.points[-1].time_from_start.nanosec * 1e-9))
        return traj

    def execute(self, traj, timeout=60.0):
        if not self.action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('JTC action server 10s 未出现 (JTC 未激活? 切换未做?)')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('JTC 拒绝轨迹目标')
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=timeout)
        if res.result() is None:
            raise RuntimeError('执行 %.0fs 未完成' % timeout)
        return res.result().result.error_code  # 0 = SUCCESS


def main(args=None):
    j2 = 0.6
    plan_timeout = 15.0
    argv = sys.argv[1:]
    while argv:
        a = argv.pop(0)
        if a == '--j2':
            j2 = float(argv.pop(0))
        elif a == '--plan-timeout':
            plan_timeout = float(argv.pop(0))
        else:
            raise SystemExit("未知参数 '%s' (--j2 --plan-timeout)" % a)
    rclpy.init(args=args)
    node = MovePlanClient()
    try:
        traj = node.plan(j2, plan_timeout)
        switch('--deactivate', 'joint_stream_controller',
               '--activate', 'joint_trajectory_controller')
        node.get_logger().info('执行窗口: JS→JTC')
        try:
            code = node.execute(traj)
            ok = (code == 0)
            node.get_logger().info('执行完成 error_code=%d %s' % (
                code, '✓' if ok else '✗'))
        finally:
            try:
                switch('--deactivate', 'joint_trajectory_controller',
                       '--activate', 'joint_stream_controller')
                node.get_logger().info('切回 JS (链恢复流式)')
            except Exception as e:
                node.get_logger().error('切回失败 (%s)。手动: ros2 control '
                    'switch_controllers --deactivate joint_trajectory_controller '
                    '--activate joint_stream_controller' % str(e).strip()[-120:])
        return 0 if ok else 1
    except Exception as e:
        node.get_logger().error(str(e))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
