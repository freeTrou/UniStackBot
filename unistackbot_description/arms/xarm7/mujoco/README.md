# xarm7 MuJoCo 资产

`xarm7.xml` — MuJoCo MJCF 模型 (2026-09-21), 管线与坑同 piper (**先读 `arms/piper/mujoco/README.md`**)。

## 与 piper 的差异

| 项 | piper | xarm7 |
|---|---|---|
| 关节 | 6 + gripper + 2 mimic 手指 | 7 (裸臂, 无 mimic → 无 equality) |
| 关节 damping | 0.1 (转换器默认) | **10/5/2 (URDF `<dynamics>` 厂商值, 转换器自动带入)** |
| 凸包互穿 | 2 对 (需 contact exclude) | **零接触, 无需 exclude** |
| 零位姿态 | 近竖直 | 近竖直 (重力矩最大 7.4 N·m @joint2, effort 限 50 余量足) |
| mesh 布局 | `../meshes/collision/` | `../meshes/xarm7/collision/` (meshdir 上提一级到 `../meshes`) |

注: `meshes/end_tool/` 资产在上游宏中本就注释未接线 (URDF 无 end_tool link), MJCF 同样没有。

## 离线验证 (2026-09-21, mujoco_env 3.8.1)

- qpos0 零接触; hold-zero 4s 下垂 max 0.9 mrad; 全关节并发 6s 稳态误差 ≤1.0 mrad
- kp 梯度 15000/15000/8000/5000/3000/2000/1000 + dampratio=1; actuatorfrcrange 50/50/30/30/30/20/20 (URDF effort)

## 再生成 (URDF 惯量/结构/限位改动后)

```bash
xacro arms/xarm7/urdf/xarm7.urdf.xacro use_ros2_control:=false use_world:=false > /tmp/xarm7_flat.urdf
ros2 run mujoco_ros2_control robot_description_to_mjcf.sh \
    -u /tmp/xarm7_flat.urdf -m arms/xarm7/mujoco/xarm7_inputs.xml \
    --scene <场景文件> -o /tmp/xarm7_mjcf -c -s
# 取 mujoco_description_formatted.xml 的身体树, mesh 换 STL (meshdir ../meshes) + 场景并入
```
