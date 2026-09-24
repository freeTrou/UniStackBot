# gen/ — IKFast 闭式解生成管线

## 输入

- `piper_canonical.urdf`：piper 裸臂（6R，无夹爪/无 ros2_control/无 world）的**规范化**展开版。
  规范化内容：joint6 origin x 的 `8.8259E-05` → `0`（88µm 垂直偏差归零——IKFast 要求结构精确；
  真实臂与规范化臂的差 = 88µm，远小于 mm 级容差，账记在决策卡 §1）。
  再生成：`xacro .../piper.urdf.xacro use_gripper:=false use_ros2_control:=false use_world:=false`
  然后重做归零替换（当前展开里该值唯一出现）。

## 执行

```bash
export http_proxy=http://10.0.3.133:7890 https_proxy=http://10.0.3.133:7890   # 拉镜像需要
bash gen/generate_in_container.sh
```

管线：podman 拉 `personalrobotics/ros-openrave` → 容器内 `urdf_to_collada` → 容器内
`openrave.py --database inversekinematics --iktype=Transform6D --iktests=200` →
生成物落 `gen/generated/`。

## 产物与门

| 产物 | 门 |
|---|---|
| `generated/analytic_piper_gen.cpp` | ①`--iktests=200` 全部通过 ②license 头过目（OpenRAVE LGPL 条款）③git 入库前人工复核无路径泄漏 |
| `generated/ikfast.h` | 与 cpp 同版本 |

## 接入（生成成功后）

1. `analytic_piper.cpp` 的 `solve()` 换闭式实现（ready 翻真），封装层保持 IkSolver 契约
2. playbook 六阶段（oracle/ssik 对拍、FK 回代、分支枚举=8、限位契约、AB 对比 DlsIk）
3. piper yaml `ik_solver: analytic_piper` 显式切换 + verify 回归
