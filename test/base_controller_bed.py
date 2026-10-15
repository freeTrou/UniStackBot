#!/usr/bin/env python3
"""base_velocity_controller 的 gz 全链断言床 (2026-10-15, 底盘域 A6; test_base_controller.sh 消费)。

五场景 (退出码 0/1, 输出 PASS:/FAIL:/WARN: 行供 shell 记账):
  1 直行   vx=0.4 → 双轮 ~4 rad/s 同号 + odom x 前进、y/yaw 微小
  2 断流   停发 2s → 双轮 |ω|<0.5 (超时 0.5s + 整形减速停车 —— 与臂域"保持"相反)
  3 原地转 wz=0.6 → 双轮 ±1.2 rad/s 反号 + odom yaw 转动
  4 超限   vx=10 → 钳位到 max_lin_x=1.0 → 轮速 ≤10.5 (不放大 10 倍)
  5 非全向 vy=0.3 → 双轮静止 (vy 诚实钳零, 不静默执行也不放大)
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class Bed(Node):

	def __init__(self):
		super().__init__('base_controller_bed')
		self.js = {}
		self.odom = None
		self.create_subscription(JointState, '/joint_states', self._on_js, 20)
		self.create_subscription(Odometry, '/odom', self._on_odom, 20)
		self.pub = self.create_publisher(Twist, '/cmd_vel',
			QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1))

	def _on_js(self, m):
		for n, v in zip(m.name, m.velocity):
			self.js[n] = v

	def _on_odom(self, m):
		self.odom = m

	def wheels(self):
		return self.js.get('left_wheel_joint'), self.js.get('right_wheel_joint')

	def spin_for(self, sec):
		t0 = time.time()
		while time.time() - t0 < sec:
			rclpy.spin_once(self, timeout_sec=0.02)

	def drive(self, vx, vy, wz, sec, hz=20):
		"""发 cmd_vel 流 (点流语义: 每条=最新目标)。"""
		dt = 1.0 / hz
		t0 = time.time()
		msg = Twist()
		msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(wz)
		n = 0
		while time.time() - t0 < sec:
			self.pub.publish(msg)
			n += 1
			rclpy.spin_once(self, timeout_sec=0.0)
			remain = t0 + n * dt - time.time()
			if remain > 0:
				time.sleep(remain)

	def settle(self, sec):
		self.spin_for(sec)
		return self.odom.pose.pose if self.odom else None


def main():
	rclpy.init()
	bed = Bed()
	rc = 0

	def check(ok, msg):
		nonlocal rc
		print(('PASS: ' if ok else 'FAIL: ') + msg)
		if not ok:
			rc = 1

	# 等链路就绪: 关节反馈 + odom 在流
	t0 = time.time()
	while (not bed.js or bed.odom is None) and time.time() - t0 < 20:
		rclpy.spin_once(bed, timeout_sec=0.1)
	check(bool(bed.js) and bed.odom is not None, '链路就绪 (/joint_states + /odom)')

	# ---- 1 直行 ----
	bed.drive(0.4, 0, 0, 4.0)
	wl, wr = bed.wheels()   # 采样紧跟 drive 末 (间隔>0.5s 会撞断流减速段)
	check(wl is not None and abs(wl - 4.0) < 0.8 and abs(wr - 4.0) < 0.8, '直行轮速 %.2f/%.2f rad/s (期望 ~4)' % (wl or -9, wr or -9))
	p = bed.settle(0.2)
	if p is not None:
		check(p.position.x > 0.8 and abs(p.position.y) < 0.2 and abs(p.orientation.z) < 0.1,
			'直行 odom x=%.2fm y=%.3f (期望 x>0.8, |y|<0.2)' % (p.position.x, p.position.y))

	# ---- 2 断流→零速 ----
	bed.spin_for(2.0)   # 停发: 超时 0.5s + 整形减速
	wl, wr = bed.wheels()
	check(abs(wl or 9) < 0.5 and abs(wr or 9) < 0.5, '断流停车 |ω|=%.2f/%.2f (期望 <0.5)' % (wl or 9, wr or 9))

	# ---- 3 原地转 ----
	bed.drive(0, 0, 0.6, 3.0)
	wl, wr = bed.wheels()
	lo, hi = sorted([abs(wl or 0), abs(wr or 0)])
	check(lo > 0.6 and hi < 1.8 and (wl or 0) * (wr or 0) < 0, '原地转反号轮速 %.2f/%.2f (期望 ±1.2)' % (wl or 0, wr or 0))
	import math
	p = bed.settle(0.2)
	if p is not None:
		yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
		check(abs(yaw) > 0.8, '原地转 odom yaw=%.2f rad (期望 |>0.8|)' % yaw)
	bed.spin_for(2.0)   # 停稳

	# ---- 4 超限钳位 ----
	bed.drive(10.0, 0, 0, 2.0)
	wl, wr = bed.wheels()
	check(abs(wl or 99) < 11.0 and abs(wr or 99) < 11.0, '超限钳位轮速 %.1f/%.1f (期望 ≤10.5, 不放大)' % (wl or 99, wr or 99))
	bed.spin_for(2.0)   # 停稳

	# ---- 5 非全向 vy 钳零 ----
	p0 = bed.settle(0.2)
	bed.drive(0, 0.3, 0, 2.0)
	wl, wr = bed.wheels()
	p1 = bed.settle(0.2)
	check(abs(wl or 9) < 0.3 and abs(wr or 9) < 0.3, 'vy 钳零轮速 %.2f/%.2f (期望 |ω|<0.3)' % (wl or 9, wr or 9))
	if p0 is not None and p1 is not None:
		check(abs(p1.position.x - p0.position.x) < 0.1, 'vy 钳零位移不动 dx=%.3f' % abs(p1.position.x - p0.position.x))

	bed.destroy_node()
	rclpy.shutdown()
	return rc


if __name__ == '__main__':
	sys.exit(main())
