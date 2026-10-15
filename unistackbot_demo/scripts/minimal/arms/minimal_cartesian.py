#!/usr/bin/env python3
"""最小笛卡尔接入教学 demo (机型无关) —— 90 行看懂笛卡尔域怎么接入 UniStackBot。

教学件 vs 控制件: 本文件只演示链路契约的最小正确用法 (拿反馈 → 切 CM →
发 2 个目标 → 切回), 零机型特化; 完整工况与防御细节看 scripts/<robot>_arm/
与 demo_cartesian.py。

链路契约 (四条):
  发  /cartesian_motion_controller/command  geometry_msgs/PoseStamped
      base 系目标位姿点流 (每条消息 = 最新目标), QoS reliable+KeepLast(1)。
      CM 内部做 IK(种子=实测) + OTG 整形, 发过去臂就平滑走过去。
  收  /cartesian_motion_controller/status  CartesianMotionStatus (20Hz, active 才有)
      arrival(到位三值: 到了/到了且停住) / position_error / last_result(IkResult 错误码) / solver_health。
  切  两控制器命令接口互斥 —— CM 接管必须显式 switch, 用完切回 JS:
      ros2 control switch_controllers --deactivate joint_stream_controller
                                        --activate cartesian_motion_controller
  读  末端链 (base→tip) 从 CM 自身参数读 —— 求解器配的链就是权威链。

前置: 任一链已启动。用法: ros2 run unistackbot_demo minimal_cartesian.py
流程: 读末端链 → 拿反馈(当前末端位姿) → 切 CM → z 走 --dz (默认抬 10cm) → 回位 → 切回 JS。
注意: 目标相对当前位姿派生 —— 重复运行会向工作空间边界累积; 偶发拒绝
(last_result=3) 或想往下走, 用 --dz 负值自愈: ros2 run ... --dz -0.15
"""

import re
import subprocess
import sys
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

from unistackbot_interface.msg import CartesianMotionStatus

CM_CMD = '/cartesian_motion_controller/command'
CM_STATUS = '/cartesian_motion_controller/status'
STEP_Z = 0.10   # 演示步幅 [m]


def end_effector_chain(node):
    """末端链 (base, tip) 读自 CM 参数 —— 求解器配置的链就是权威链 (机型无关)。"""
    for node_name, names in (
        ('/cartesian_motion_controller', ['base_link', 'tip_link']),
        ('/controller_manager', ['cartesian_motion_controller.base_link',
                                 'cartesian_motion_controller.tip_link']),
    ):
        try:
            cli = node.create_client(GetParameters, node_name + '/get_parameters')
            if not cli.wait_for_service(timeout_sec=3.0):
                continue
            fut = cli.call_async(GetParameters.Request(names=names))
            rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
            if fut.result() and fut.result().values[0].string_value:
                return (fut.result().values[0].string_value,
                        fut.result().values[1].string_value)
        except Exception:
            continue
    raise RuntimeError('读 CM base/tip 失败 (CM 未加载? 诊断: ros2 param list /controller_manager)')


class MinimalCartesianDemo(Node):

    def __init__(self):
        super().__init__('minimal_cartesian')
        self.status = {}
        self.sent = None   # 对账锚点: 最近发布的目标

    @staticmethod
    def _controller_states():
        """list_controllers → {name: (type, state)} (剥 ANSI)。"""
        import subprocess
        try:
            r = subprocess.run(['ros2', 'control', 'list_controllers'],
                               capture_output=True, text=True, timeout=15)
        except subprocess.TimeoutExpired:
            raise RuntimeError('list_controllers 超时 —— 链未启动或 CLI 缺失?'
                ' 先起链 (ros2 launch ...); CLI 缺失装 ros-humble-ros2controlcli')
        states = {}
        for line in r.stdout.splitlines():
            clean = re.sub(r'\x1b\[[0-9;]*m', '', line.strip())
            parts = clean.split()
            if len(parts) >= 3:
                states[parts[0]] = (parts[1], parts[-1])
        return states

    @staticmethod
    def _js_names(states, want_state):
        """'@js' 通配: JointStream 实例按类型发现, 命名任意 (单块机型
        joint_stream_controller / 多实例机型 js_arms+js_torso), 形态盲。"""
        return sorted(n for n, (t, s) in states.items()
                      if s == want_state and 'JointStreamController' in t)

    def ensure_controllers(self, activate, deactivate=(), retries=7):
        """进入声明/退出礼让 (幂等): 先查当前态, 只对需要变化的发 switch; 名单里
        '@js' = 全部 JointStream 实例 (按类型发现)。activate 已 active 的控制器会
        被 CM 硬拒 (not inactive) —— 幂等由状态查询实现。"""
        for attempt in range(retries):
            states = self._controller_states()
            if not states:
                raise RuntimeError('list_controllers 无输出 —— 链未启动? 先 ros2 launch 再跑 demo')
            need_deact = (self._js_names(states, 'active') if '@js' in deactivate
                          else [c for c in deactivate if states.get(c, ('', ''))[1] == 'active'])
            need_act = (self._js_names(states, 'inactive') if '@js' in activate
                        else [c for c in activate if states.get(c, ('', ''))[1] != 'active'])
            if not need_deact and not need_act:
                return
            if need_deact:
                # 单 flag 多值 (2026-10-14 实锤): --deactivate/--activate 是 nargs='*',
                # 重复同名 flag 被 argparse 覆盖只留最后一个 —— "Successfully switched"
                # 假成功, 前面的控制器根本没停。正确写法: --deactivate a b
                subprocess.run(['ros2', 'control', 'switch_controllers', '--deactivate'] + need_deact,
                               capture_output=True, text=True, timeout=15)
                time.sleep(0.5)   # 接口释放落定
            if need_act:
                s = subprocess.run(['ros2', 'control', 'switch_controllers', '--activate'] + need_act,
                                   capture_output=True, text=True, timeout=15)
                if s.returncode == 0:
                    return
            self.get_logger().warn('switch 未就绪 (attempt %d/%d), 5s 重试' % (attempt + 1, retries))
            time.sleep(5.0)
        raise RuntimeError('switch 失败: %s' % s.stderr.strip()[-120:])

    def goto(self, pub, tf, dz, tag):
        """走一个点: 流式插值 (smoothstep 起停零速度) → 等收敛或拒绝。"""
        for i in range(50):
            a = (i + 1) / 50.0
            a = a * a * (3.0 - 2.0 * a)
            t = PoseStamped()
            t.header.frame_id = tf.header.frame_id
            t.pose.position.z = tf.transform.translation.z + dz * a
            t.pose.orientation = tf.transform.rotation
            self.status.clear()   # 只认发布之后的新 status
            self.sent = t.pose.position.z
            pub.publish(t)
            time.sleep(0.02)
        t0 = time.time()
        while time.time() - t0 < 15.0:
            rclpy.spin_once(self, timeout_sec=0.05)
            # arrival=SETTLED (新正字段): 到了且停住 —— converged 已 deprecated
            if self.status.get('arrival') == 2 and self.status.get('tz') is not None and \
                abs(self.status['tz'] - self.sent) < 1e-9:
                self.get_logger().info('%s: 收敛 err=%.3fmm' % (tag, self.status['err'] * 1000.0))
                return
            if time.time() - t0 > 1.5 and self.status.get('res', 0) != 0:
                self.get_logger().warn('%s: 拒绝 (last_result=%s, solver_health=%d)'
                    ' —— 错误码表见 unistackbot_interface/ik_result.hpp' % (
                    tag, self.status['res'], self.status.get('sh', 1)))
                return
        self.get_logger().warn('%s: 15s 未收敛' % tag)

    def run(self, dz=0.10):
        # 0. 契约认知: 末端链读自 CM 参数 (权威链)
        base, tip = end_effector_chain(self)
        self.get_logger().info('末端链 (CM 参数): %s -> %s' % (base, tip))

        # 1. 第一步 = 切换自己需要的控制器 (与 JS 命令接口互斥——同一批 position
        #    接口至多一个 active 持有; 笛卡尔域的第一动作就是拿到 CM 控制权)
        self.ensure_controllers(activate=['cartesian_motion_controller'],
                                deactivate=['@js'])
        self.get_logger().info('CM 接管: z 走 %+.0fcm (单程; 多点位看 r1_7a_demo_cartesian)' % (dz * 100))
        pub = self.create_publisher(PoseStamped, CM_CMD,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.create_subscription(CartesianMotionStatus, CM_STATUS, self._on_status, 10)

        # 2. 拿反馈: 当前末端位姿 (TF, base 系) —— 目标的派生基准 (反馈流与 active 无关)
        tfbuf = Buffer()
        TransformListener(tfbuf, self)
        t0 = time.time()
        while time.time() - t0 < 5.0 and \
            not tfbuf.can_transform(base, tip, rclpy.time.Time()):
            rclpy.spin_once(self, timeout_sec=0.1)
        tf = tfbuf.lookup_transform(base, tip, rclpy.time.Time())
        tf.header.frame_id = base
        p = tf.transform.translation
        self.get_logger().info('当前末端: z=%.3f m' % p.z)

        try:
            # 3. 发一个目标 (相对当前位姿) —— 这就是笛卡尔接入的全部: 拿反馈、
            #    派生目标、发出去。偶发拒绝 (ITERATION_LIMIT) = 错误处理教学点;
            #    撞工作空间边界就往回走: ros2 run ... --dz -0.15
            self.goto(pub, tf, dz, 'z %+.0fcm' % (dz * 100))
        finally:
            # 4. 切回 JS (命令接口让还给关节域; '@js' 按类型发现, 命名任意)
            self.ensure_controllers(activate=['@js'],
                                    deactivate=['cartesian_motion_controller'])
            self.get_logger().info('已切回 joint_stream')

    def _on_status(self, msg):
        self.status.update(err=msg.position_error, arrival=msg.arrival,
                           res=msg.last_result, sh=msg.solver_health,
                           tz=msg.target_pose.position.z)


def main():
    dz = 0.10
    argv = sys.argv[1:]
    while argv:
        a = argv.pop(0)
        if a == '--dz':
            dz = float(argv.pop(0))   # 负值 = 往下走 (撞边界后的自愈入口)
        else:
            raise SystemExit("未知参数 '%s' (支持: --dz <m>)" % a)
    rclpy.init()
    node = MinimalCartesianDemo()
    try:
        node.run(dz)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
