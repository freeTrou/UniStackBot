#!/usr/bin/env python3
"""给 JTC 发送一段往返演示轨迹 (区间 35% -> 65% -> 20% -> 回零位附近), 机型无关。

关节表实时读自 /joint_trajectory_controller 参数 (即 <robot>_controllers.yaml 的 joints),
限位/最大速度解析自 robot_state_publisher 的 robot_description (URDF <ros2_control> 块),
路径点按限位区间百分比推算 —— 本脚本不含任何机型硬编码。

前置: control.launch.py / ign.launch.py / gazebo.launch.py 任一已启动,
且 joint_trajectory_controller 处于 active。

用法: ros2 run unistackbot_bringup demo_motion.py
"""

import math
import xml.etree.ElementTree as ET

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class DemoMotion(Node):

	def __init__(self):
		super().__init__('demo_motion')
		self._client = ActionClient(
			self, FollowJointTrajectory,
			'/joint_trajectory_controller/follow_joint_trajectory')

	def _get_param(self, node_name, name):
		"""经参数服务读远端节点参数 (Humble rclpy 无 SyncParameterClient, 手工建客户端)。"""
		cli = self.create_client(GetParameters, node_name + '/get_parameters')
		if not cli.wait_for_service(timeout_sec=5.0):
			raise RuntimeError('参数服务 %s/get_parameters 5 秒内未出现 (链路未起?)' % node_name)
		req = GetParameters.Request()
		req.names = [name]
		fut = cli.call_async(req)
		rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
		return fut.result().values[0]

	def _fetch_contract(self):
		"""关节表 + 每关节 (min, max, max_velocity), 全部来自运行中的链路, 不本地写死。"""
		jv = self._get_param('/joint_trajectory_controller', 'joints')
		if not jv.string_array_value:
			raise RuntimeError('读取 /joint_trajectory_controller 的 joints 参数失败')
		joints = list(jv.string_array_value)

		uv = self._get_param('/robot_state_publisher', 'robot_description')
		if not uv.string_value:
			raise RuntimeError('读取 robot_state_publisher 的 robot_description 失败')
		urdf = uv.string_value
		lim = {}
		for j in ET.fromstring(urdf).iter('joint'):
			if j.get('name') is None or j.find('command_interface') is None:
				continue
			ps = {p.get('name'): p.text for p in j.findall('param')}
			if 'min' in ps and 'max' in ps:
				lim[j.get('name')] = (
					float(ps['min']), float(ps['max']),
					float(ps.get('max_velocity', '1.0')))
		missing = [n for n in joints if n not in lim]
		if missing:
			raise RuntimeError('关节缺限位声明: %s (URDF ros2_control 块需 min/max)' % missing)
		return joints, [lim[n] for n in joints]

	def run(self):
		if not self._client.wait_for_server(timeout_sec=10.0):
			self.get_logger().error(
				'10 秒内未找到 joint_trajectory_controller 的 action 服务器, 可能原因:\n'
				'  1) 链路未启动: ros2 launch unistackbot_bringup control.launch.py robot:=<机型>\n'
				'  2) 控制器未激活: 等 launch 日志出现两行 "Configured and activated"\n'
				'  3) 有残留进程冲突: ros2 run unistackbot_gazebo gz_clean.sh')
			return

		joints, lims = self._fetch_contract()
		self.get_logger().info('关节表 (读自控制器): %s' % joints)

		lo = [l for l, _, _ in lims]
		hi = [h for _, h, _ in lims]
		mv = [v for _, _, v in lims]
		frac = lambda f: [l + f * (h - l) for l, h in zip(lo, hi)]
		# 往返序列: 区间 35% -> 65% -> 20% -> 零位(夹进限位)
		seq = [frac(0.35), frac(0.65), frac(0.20),
		       [min(max(0.0, l), h) for l, h in zip(lo, hi)]]

		goal = FollowJointTrajectory.Goal()
		goal.trajectory.joint_names = joints
		t = 0.0
		prev = [0.0] * len(joints)
		for positions in seq:
			# 段时长按最慢关节推算 (留 50% 余量), 下限 2s 上限 8s
			travel = max(abs(p - q) / v for p, q, v in zip(positions, prev, mv))
			t += max(2.0, min(8.0, travel * 1.5 + 1.0))
			point = JointTrajectoryPoint()
			point.positions = [float(p) for p in positions]
			point.time_from_start.sec = int(math.ceil(t))
			goal.trajectory.points.append(point)
			prev = positions
		total = goal.trajectory.points[-1].time_from_start.sec

		self.get_logger().info('发送轨迹 (%d 个路径点, 约 %d 秒)...' % (len(seq), total))
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
	node = DemoMotion()
	try:
		node.run()
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
