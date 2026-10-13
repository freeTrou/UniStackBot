#!/usr/bin/env python3
"""最小关节接入教学 demo (机型无关) —— 60 行看懂关节域怎么接入 UniStackBot。

教学件 vs 控制件: 本文件只演示链路契约的最小正确用法 (读反馈 → 发目标 → 回位),
零机型特化; 完整工况与防御细节看 scripts/<robot>_arm/ 与 demo_motion.py。

链路契约 (就这两条):
  发  /joint_stream_controller/command  unistackbot_interface/JointCommand
      CSP 点流: 每条消息 = 最新目标; joint_names 必须列全全部命令关节。
      QoS reliable+KeepLast(1)。JS 内建 ruckig 整形 —— 一条目标发过去臂就
      平滑走过去; 停止发送 = 臂保持在最后目标 (断流语义)。
  收  /joint_states  sensor_msgs/JointState
      关节真值; name 与数值按索引对应但顺序无保证 —— 必须按名对齐。

前置: 任一链已启动。用法: ros2 run unistackbot_demo minimal_joint.py
流程: 解析命令关节 → 拿反馈 → J1 抬 0.3 rad → 回出发位。完。
"""

import time
import xml.etree.ElementTree as ET

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from unistackbot_interface.msg import JointCommand

CMD_TOPIC = '/joint_stream_controller/command'
STEP = 0.3   # 演示步幅 [rad] (小步; 越界会被 JS 限位 clamp 兜底)


def command_joints(node):
    """命令关节表: 解析 /robot_description 的 <ros2_control> 块 (带 position
    命令接口的关节)。mimic 关节由 JS 内部推导, 不在此列。"""
    cli = node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
    if not cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError('链路未启动?')
    fut = cli.call_async(GetParameters.Request(names=['robot_description']))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    urdf = fut.result().values[0].string_value
    names = []
    for j in ET.fromstring(urdf).iter('joint'):
        ci = j.find('command_interface')
        if j.get('name') and ci is not None and ci.get('name') == 'position' \
            and j.find('param[@name="mimic"]') is None:
            names.append(j.get('name'))
    return names


class MinimalJointDemo(Node):

    def __init__(self):
        super().__init__('minimal_joint')

    def ensure_controllers(self, activate, deactivate, retries=7):
        """进入声明 (幂等): 先查当前态, 只对需要变化的发 switch。
        注意 activate 已 active 的控制器会被 CM 硬拒 (not inactive) ——
        幂等必须由状态查询实现, 不能盲发。链刚起时 spawner 窗口内重试。"""
        import re
        import subprocess
        for attempt in range(retries):
            r = subprocess.run(['ros2', 'control', 'list_controllers'],
                               capture_output=True, text=True, timeout=15)
            states = {}
            for line in r.stdout.splitlines():
                clean = re.sub(r'\x1b\[[0-9;]*m', '', line.strip())   # 剥 ANSI 色码
                parts = clean.split()
                if len(parts) >= 3:
                    states[parts[0]] = parts[-1]
            args = []
            for c in deactivate:
                if states.get(c) == 'active':
                    args += ['--deactivate', c]
            for c in activate:
                if states.get(c) != 'active':
                    args += ['--activate', c]
            if not args:
                return   # 已达目标态 = 幂等成功
            s = subprocess.run(['ros2', 'control', 'switch_controllers'] + args,
                               capture_output=True, text=True, timeout=15)
            if s.returncode == 0:
                return
            self.get_logger().warn('switch 未就绪 (attempt %d/%d), 5s 重试' % (attempt + 1, retries))
            time.sleep(5.0)
        raise RuntimeError('switch 失败: %s' % s.stderr.strip()[-120:])

    def get_feedback(self, names):
        """拿反馈: 等一帧 /joint_states, 按名对齐。"""
        got = {}
        sub = self.create_subscription(JointState, '/joint_states',
            lambda m: got.update(zip(m.name, m.position)), 10)
        t0 = time.time()
        while any(n not in got for n in names) and time.time() - t0 < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return {n: got.get(n, 0.0) for n in names}

    def run(self):
        # 第一步 = 切换自己需要的控制器 (进入声明, 幂等): 关节域入口是 JS。
        # 别假设 JS"默认在" (上一轮笛卡尔 demo 可能切走了); 也不盲 activate
        # 已 active 的 JS (会被 CM 拒) —— ensure 内部查态后只切需要变的。
        self.ensure_controllers(activate=['joint_stream_controller'],
                                deactivate=['cartesian_motion_controller'])

        names = command_joints(self)
        self.get_logger().info('命令关节 (%d): %s' % (len(names), names))

        fb = self.get_feedback(names)
        self.get_logger().info('当前关节: %s' % {k: round(v, 3) for k, v in fb.items()})

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        pub = self.create_publisher(JointCommand, CMD_TOPIC, qos)

        def send(position, hold=2.5):
            m = JointCommand()
            m.mode = JointCommand.MODE_CSP
            m.joint_names = names
            m.position = position
            pub.publish(m)
            time.sleep(hold)   # 一条消息发过去, ruckig 平滑走完, hold 等到位

        start = [fb[n] for n in names]
        stepped = list(start)
        stepped[0] += STEP   # J1 抬一小步 (越界会被 JS 限位 clamp 兜底)
        self.get_logger().info('%s[0] += %.1f → 回出发位' % (names[0], STEP))
        send(stepped)
        send(start)
        self.get_logger().info('完成 (回到出发位)')


def main():
    rclpy.init()
    node = MinimalJointDemo()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
