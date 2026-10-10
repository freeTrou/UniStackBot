#!/usr/bin/env python3
"""点位序列回放 (命令域② 变体): 点位资产 json → JS 关节流。

场景: 外部工程采集的关节点位序列 (遥操采样/示教/走廊轨迹) 在仿真链回放验证。
设计: 与 demo_motion 同款契约消费 (URDF 自持解析关节表/限位/mimic) + 序列语义:
  - --seq 逗号分隔: 字面点名=锚点(单发+settle), glob 项=连流段(自然排序, 按 --hz);
    首项恒为锚点 (settle 吸收起跑跳变, ruckig 填充)
  - --seq "@序列名" 用 v1 资产显式序列: 首项锚点, 余项全部连流——资产已显式有序,
    分段不依赖命名约定 (2026-10-10 退役旧启发式"相邻点数字后缀判连流":
    非数字命名的资产每点都吃 settle, 规范 v1 反例)
  - 序列外关节 (另一臂/夹爪) 持实测当前值
  - 每点限位钳位 (采集噪声贴限位常见, 计数报告)
  - meta.joint_names 非空时与链上侧关节逐位校验 (q→关节映射是位置性的, 序错=回放错)

用法: ros2 run unistackbot_demo demo_points_playback.py
        --points-file <v1.json> --side left
        --seq "@corridor_left" 或 "transit_l,to_cupdrop_*,cup_drop" [--hz 20] [--settle 1.5]
"""

import glob as _glob
import json
import sys
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from unistackbot_interface.msg import JointCommand
import xml.etree.ElementTree as ET


def fetch_contract(node):
    """直控关节表+限位+mimic (demo_motion 同口径)。"""
    cli = node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
    if not cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError('参数服务未出现 (链未起?)')
    req = GetParameters.Request()
    req.names = ['robot_description']
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    uv = fut.result().values[0]
    joints, lims, mimics = [], [], []
    for j in ET.fromstring(uv.string_value).iter('joint'):
        if j.get('name') is None or j.find('command_interface') is None:
            continue
        if j.find('command_interface').get('name') != 'position':
            continue
        name = j.get('name')
        ps = {p.get('name'): p.text for p in j.findall('param')}
        if 'mimic' in ps:
            mimics.append((name, ps['mimic'],
                           float(ps.get('multiplier', '1')), float(ps.get('offset', '0'))))
        elif 'min' in ps and 'max' in ps:
            joints.append(name)
            lims.append((float(ps['min']), float(ps['max'])))
        else:
            raise RuntimeError("命令关节 '%s' 无 min/max 也无 mimic" % name)
    masters = set(joints)
    for name, master, _, _ in mimics:
        if master not in masters:
            raise RuntimeError("mimic '%s' 主关节不在表" % name)
    return joints, lims, mimics


def _natural_key(k):
    """自然排序键: 尾部数字按数值 (字典序会把 100..145 排到 10 前面、99 排最后
    —— 2026-10-09 走廊乱序事故根因: 点序被打乱成大跳, 末端停在字典序末点)。"""
    head = k.rstrip('0123456789')
    tail = k[len(head):]
    return (head, int(tail) if tail else -1)


def expand_seq(spec, points):
    """--seq 展开: [(点名, 连流?), ...]; 字面名=锚点, glob 项=连流段
    (glob 展开按自然排序: 尾部数字按值)。"""
    out = []
    for item in spec.split(','):
        item = item.strip()
        if not item:
            continue
        if any(c in item for c in '*?['):
            hits = sorted((k for k in points if _glob.fnmatch.fnmatch(k, item)),
                          key=_natural_key)
            if not hits:
                raise SystemExit("glob '%s' 无匹配点" % item)
            out.extend((h, True) for h in hits)
        else:
            if item not in points:
                raise SystemExit("点 '%s' 不在文件中" % item)
            out.append((item, False))
    return out


def main(args=None):
    pf = side = seq = None
    hz = 20.0
    settle = 1.5
    argv = sys.argv[1:]
    while argv:
        a = argv.pop(0)
        if a == '--points-file':
            pf = argv.pop(0)
        elif a == '--side':
            side = argv.pop(0)
        elif a == '--seq':
            seq = argv.pop(0)
        elif a == '--hz':
            hz = float(argv.pop(0))
        elif a == '--settle':
            settle = float(argv.pop(0))
        else:
            raise SystemExit("未知参数 '%s'" % a)
    if not pf or not side or not seq:
        print('--points-file/--side/--seq 必填')
        return 2
    data = json.load(open(pf))
    # 数据录制规范 v1 (unistackbot_tool/README §3): {meta, points, sequences};
    # v0 (裸 {side: {...}}) 双兼容
    sequences = data.get('sequences', {})
    if 'points' in data:
        points = data['points'].get(side)
    else:
        points = data.get(side)
    if not points:
        print("文件无 '%s' 节或为空" % side)
        return 2
    meta = data.get('meta', {})
    if meta:
        print('meta: robot=%s side=%s hz=%s source=%s' % (
            meta.get('robot'), meta.get('side'), meta.get('hz'), meta.get('source')))
        if meta.get('notes'):
            print('notes: %s' % meta['notes'])

    rclpy.init(args=args)
    node = Node('demo_points_playback')
    try:
        held = {}
        node.create_subscription(JointState, '/joint_states',
                                 lambda m: held.update(zip(m.name, m.position)), 10)
        joints, lims, mimics = fetch_contract(node)
        dim = len(next(iter(points.values())))
        # 侧关节 = 关节表中前缀匹配 side 的前 dim 个直控关节 (left_joint1..6)
        side_joints = [j for j in joints if j.startswith(side + '_')][:dim]
        if len(side_joints) != dim:
            raise SystemExit("侧关节不匹配: 文件 %d 维 vs 链上 %s 共 %d 个"
                             % (dim, side, len(side_joints)))
        mjn = list(meta.get('joint_names') or [])
        if mjn and mjn != side_joints:
            raise SystemExit('meta.joint_names %s\n与链上侧关节 %s 不一致——映射按位, 序错=回放错'
                             % (mjn, side_joints))
        node.get_logger().info('关节表 %d 直控 (本侧: %s), mimic %d'
                               % (len(joints), side_joints, len(mimics)))

        if seq.startswith('@'):
            sname = seq[1:]
            if sname not in sequences:
                print("序列 '%s' 不在文件 sequences 节" % sname)
                return 2
            # v1 显式序列 (显式有序, 规范 v1 硬规则): 首项锚点吸收起跑跳变, 余项连流
            pairs = [(n, i > 0) for i, n in enumerate(sequences[sname])]
        else:
            pairs = expand_seq(seq, points)
        n_anchor = sum(1 for _, st in pairs if not st)
        node.get_logger().info('序列 %d 点 (锚点 %d + 连流 %d)'
                               % (len(pairs), n_anchor, len(pairs) - n_anchor))

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        pub = node.create_publisher(JointCommand,
                                    '/joint_stream_controller/command', qos)
        t0 = time.time()
        while (len(held) < len(joints)) and time.time() - t0 < 3.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(held) < len(joints):
            raise RuntimeError('/joint_states 未齐 (3s)')

        sent = 0
        clamped = 0

        def publish_full(target_map):
            nonlocal sent, clamped
            m = JointCommand()
            m.mode = JointCommand.MODE_CSP
            m.joint_names = list(joints) + [mm[0] for mm in mimics]
            pos = []
            for j, (lo, hi) in zip(joints, lims):
                v = target_map.get(j, held.get(j, 0.0))
                c = min(max(v, lo), hi)
                if c != v:
                    clamped += 1
                pos.append(c)
            for _n, master, k, off in mimics:
                pos.append(k * pos[joints.index(master)] + off)
            m.position = pos
            pub.publish(m)
            sent += 1

        period = 1.0 / hz
        for i, (name, streams) in enumerate(pairs):
            q = points[name]
            tgt = dict(held)
            tgt.update({sj: v for sj, v in zip(side_joints, q)})
            if streams and i > 0:
                publish_full(tgt)
                # spin 必须在流式循环内: RELIABLE 发布器不 spin 无法处理 ACK/重传,
                # 写入端发到一定量后停摆, JS 停在最后收到的一条 (2026-10-09 实锤,
                # 三个回放全部冻结于同一消息序号)
                nxt = time.time() + period
                rclpy.spin_once(node, timeout_sec=0)
                time.sleep(max(0.0, nxt - time.time()))
            else:
                # 首项/锚点: 单发 + settle
                publish_full(tgt)
                time.sleep(settle)
        # 末点收敛等待 (断流语义 2026-10-09: 不刹停, JS 自行收敛到最后收到的点
        # 保持) —— 纯等待观测, 无需补发
        final = points[pairs[-1][0]]
        t0c = time.time()
        converged = False
        while time.time() - t0c < 30.0:
            errs = [abs(held.get(sj, 9) - v) for sj, v in zip(side_joints, final)]
            if errs and max(errs) < 0.02:
                converged = True
                break
            rclpy.spin_once(node, timeout_sec=0.02)
            time.sleep(0.03)
        errs = [abs(held.get(sj, 9) - v) for sj, v in zip(side_joints, final)]
        node.get_logger().info('汇总: 发 %d 条, 钳位 %d 次, 末点误差 max=%.4f rad (%s)' % (
            sent, clamped, max(errs) if errs else -1,
            '收敛✓' if converged else '30s 未全敛 (如实报)'))
        return 0
    except Exception as e:
        node.get_logger().error(str(e))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
