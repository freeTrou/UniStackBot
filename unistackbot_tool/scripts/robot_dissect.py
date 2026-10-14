#!/usr/bin/env python3
"""机型结构解剖器 (URDF 导入流程 S1 的执行器, 2026-10-14 立项固化 —— 此前是 /tmp 一次性脚本)。

用法: robot_dissect.py <robot.urdf> [--capacity 16]
输出: 关节分类表 / 类型分布 / 轮式预警 (continuous) / 直线关节预警 (prismatic 单位米)
      / 容量核算 (>N 关节提示分实例) / 链拓扑 (base→tip 候选链) / 指纹登记片段草稿

定位: unistackbot_tool = 工具程序容器 (用户裁决); 解剖结果是 acceptance.yaml
指纹登记与四件式控制器分割的数据源。死档案不参与 (那是 original/ 的事)。
"""

import sys
import xml.etree.ElementTree as ET


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    argv = sys.argv[1:]
    path = argv[0]
    capacity = 16
    if '--capacity' in argv:
        capacity = int(argv[argv.index('--capacity') + 1])

    root = ET.parse(path).getroot()
    joints = list(root.iter('joint'))
    links = list(root.iter('link'))

    # ---- 分类 (按名称语义 + 类型; 轮式新维度 2026-10-14) ----
    buckets = {'wheel': [], 'torso': [], 'arm': [], 'gripper': [], 'head_sensor': [], 'fixed': [], 'other': []}
    types = {}
    for j in joints:
        t = j.get('type', '?')
        types[t] = types.get(t, 0) + 1
        n = (j.get('name') or '').lower()
        if t == 'fixed':
            buckets['fixed'].append(j.get('name'))
        elif any(k in n for k in ('wheel', 'caster', 'servo', 'steering', 'drive')):
            buckets['wheel'].append((j.get('name'), t))
        elif any(k in n for k in ('torso', 'waist', 'lift', 'column', 'spine', 'waist')):
            buckets['torso'].append((j.get('name'), t))
        elif any(k in n for k in ('arm', 'shoulder', 'elbow', 'wrist')):
            buckets['arm'].append(j.get('name'))
        elif any(k in n for k in ('gripper', 'finger', 'suction', 'claw', 'active_joint', 'passive_joint')):
            buckets['gripper'].append((j.get('name'), t))
        elif any(k in n for k in ('head', 'camera', 'zed', 'lidar', 'imu', 'zedm', 't265')):
            buckets['head_sensor'].append(j.get('name'))
        elif j.get('name') and len(j.get('name')) <= 4 and j.get('name')[0] in 'JLK' and j.get('name')[1:].isdigit():
            buckets['arm'].append(j.get('name'))   # 宇树式命名 J1-J7 (短大写字母+纯数字)
        else:
            buckets['other'].append((j.get('name'), t))

    print('== %s 解剖报告 ==' % path.split('/')[-1])
    print('joint 总数 %d | link 总数 %d | 类型分布 %s' % (len(joints), len(links), types))

    print('\n[控制域分类]')
    print('  轮 (velocity 接口, 生态件域): %s' % (buckets['wheel'] or '无'))
    print('  躯干/腰 (js_torso 域): %s' % (buckets['torso'] or '无'))
    print('  臂 (js_arms 域): %d 个 %s' % (len(buckets['arm']), sorted(buckets['arm'])[:16]))
    print('  夹爪: %s' % (buckets['gripper'] or '无'))
    print('  头/传感器 (fixed 挂点, 不进控制域): %d 个' % len(buckets['head_sensor']))
    print('  未归类 (人工审): %s' % (buckets['other'] or '无'))

    # ---- 预警 ----
    warns = []
    cont = [n for n, t in buckets['wheel'] if t == 'continuous']
    if cont:
        warns.append('轮 continuous×%d —— 位置无意义, 底盘走 velocity 接口 (生态件); JS 只认 position, 物理进不了' % len(cont))
    pris = [(j.get('name')) for j in joints if j.get('type') == 'prismatic']
    if pris:
        warns.append('prismatic×%d %s —— 单位米非弧度 (限位/速度/步长参数与消费者勿假设 rad)' % (len(pris), pris[:4]))
    n_arm = len(buckets['arm'])
    if n_arm > capacity:
        warns.append('臂关节 %d > 容量 %d —— 超限! 双臂拆实例或减关节' % (n_arm, capacity))
    n_all_ctrl = n_arm + len(buckets['torso']) + len(buckets['gripper'])
    if capacity < n_all_ctrl <= capacity + len(buckets['torso']):
        warns.append('臂+夹爪 %d 超 %d 但腰拆出后可容 —— 四件式: 腰独立实例' % (n_all_ctrl, capacity))
    if buckets['other']:
        warns.append('未归类关节 %d 个需人工归类后重跑' % len(buckets['other']))
    print('\n[预警]')
    for w in warns or ['无']:
        print('  ⚠ %s' % w)

    # ---- 链拓扑 (root link 出发找最深串 = 候选臂链) ----
    child_of = {}
    children = {}
    all_children = set()
    for j in joints:
        p, c = j.find('parent'), j.find('child')
        if p is None or c is None or j.get('type') == 'fixed':
            if p is not None and c is not None:
                child_of[c.get('link')] = (p.get('link'), j.get('type'))
            continue
        children.setdefault(p.get('link'), []).append((c.get('link'), j.get('name')))
        all_children.add(c.get('link'))
    roots = [l.get('name') for l in links if l.get('name') not in all_children and children.get(l.get('name'))]
    print('\n[拓扑] root link: %s' % roots)

    def deepest(node, path, jpath):
        best = (len(jpath), path, jpath)
        for c, jn in children.get(node, []):
            r = deepest(c, path + [c], jpath + [jn])
            if r[0] > best[0]:
                best = r
        return best

    for rt in roots[:2]:
        n, path, jpath = deepest(rt, [rt], [])
        print('  %s 最深运动链 (%d 关节): %s' % (rt, n, ' → '.join(jpath[:8]) + ('…' if n > 8 else '')))
        print('    末端 link: %s (CM tip 候选; base = %s)' % (path[-1], rt))

    # ---- 指纹登记草稿 ----
    n_sub = len(child_of) + 1
    print('\n[acceptance.yaml 指纹草稿 (人工核对后登记)]')
    print('  subchain_count: %d  # link 数 (verify 展开实测为准)' % len(links))
    print('  joints_by_chain: {mock: [%s], ign: 同, mujoco: 同}  # 待四件式分割后定' %
          ','.join(sorted(buckets['arm'])[:7]))
    print('  plugins: {mock: unistackbot_sim_control/SimControlHardware,')
    print('            ign: gz_ros2_control/GazeboSimSystem, mujoco: mujoco_ros2_control/MujocoSystemInterface}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
