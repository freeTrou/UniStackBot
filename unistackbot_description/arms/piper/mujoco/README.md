# piper MuJoCo 资产 (阶段1 MuJoCo 链)

`piper.xml` — MuJoCo MJCF 模型, 由 URDF 转换 + 手工策展 (2026-09-20)。mesh 直接引用
`../meshes/collision/` 的 STL (零复制资产, 几何单一事实源)。

## 文件

| 文件 | 作用 |
|---|---|
| `piper.xml` | 最终 MJCF (身体树/惯量/限位来自 URDF, 执行器/equality/场景手工并入) |
| `piper_inputs.xml` | 转换器输入: option/defaults/执行器/equality (再生成时用) |
| `scene_info.xml` | 转换器输入: 场景 (地板/灯光/statistic) |

## 再生成 (URDF 惯量/限位/结构改动后)

```bash
# 1. 平面 URDF (纯运动学, ros2_control 块不需要)
xacro arms/piper/urdf/piper.urdf.xacro use_gripper:=true use_ros2_control:=false \
    use_world:=false > /tmp/piper_flat.urdf

# 2. 转换 (mujoco_ros2_control 自带工具; 首次运行自建 venv ~/.ros/ros2_control/.venv)
ros2 run mujoco_ros2_control robot_description_to_mjcf.sh \
    -u /tmp/piper_flat.urdf -m arms/piper/mujoco/piper_inputs.xml \
    --scene arms/piper/mujoco/scene_info.xml -o /tmp/piper_mjcf -c -s

# 3. 手工策展: 取 /tmp/piper_mjcf/mujoco_description_formatted.xml 的身体树,
#    mesh 换 STL (meshdir ../meshes/collision/), 场景并入单文件,
#    执行器/equality 已由 inputs 注入。OBJ 美化路线 (材质/贴图) 备选未采用 (32MB)。

# 4. 离线验证 (mujoco_env conda 环境)
python /tmp/validate_piper_mjcf.py   # 零位保持 / mimic 收敛 / 全关节跟踪
```

## 策展决策记录

- **equality 方向坑 (MuJoCo 3.x 实测)**: joint equality 约束实为 `joint1 = poly(joint2)`
  (第一关节是**因变量**), 与官方文档文字表述相反。demo 的 `polycoef="0 -1"` 自反
  (R=-L ↔ L=-R) 故两个方向都对, 未暴露此坑; 我们的 0.5/-0.5 系数必须把主关节
  `gripper` 放 `joint2`。验证法: 手工设 qpos 后 `mj_forward` 看 `efc_pos`。
- **凸包接触排除**: mesh 碰撞走凸包, `base_link`↔`link1` (世界体-子体不被自动过滤)
  与两手指 (兄弟体) 互穿会卡死关节 → 显式 `<contact><exclude>`。qpos0 实测零接触。
- **mimic 拓扑**: `gripper` 主关节带 position 执行器, 手指 passive + equality 耦合
  (对齐 mujoco_ros2_control demo 的手指先例)。ros2_control 侧手指无 command_interface,
  JointStream 解析 URDF 后自动只管 7 个主关节。
- **增益起步值**: kp = 1500/1500/1000/500/300/200 (臂) + 200 (gripper), dampratio=1,
  joint damping/frictionloss=0.1 (转换器按 URDF effort 自动加 actuatorfrcrange)。
  全关节并发稳态误差 ≤5 mrad (离线验证), 上链后按跟踪调。
- **timestep**: 默认 0.002 (500Hz) 与控制器 update_rate 对齐。
