# unistackbot_bringup

顶层启动包：拉起整机节点、加载控制器配置、编排层诊断。

## 用法

```bash
# 统一入口: chain 一参切三链 (mock|gz|mujoco), 其余参数透传
ros2 launch unistackbot_bringup sim.launch.py chain:=mock robot:=piper

# mock 链（standalone controller_manager + SimControlHardware, 无仿真器）
ros2 launch unistackbot_bringup control.launch.py robot:=piper   # use_rviz:=true|false

ros2 run unistackbot_bringup demo_motion.py    # 演示轨迹 (前置: 任一链已启动, JS active)
```

gz/mujoco 链的 launch 在 `unistackbot_gazebo` / `unistackbot_mujoco`。

## 关键文件

| 文件 | 说明 |
|---|---|
| `launch/sim.launch.py` | 统一入口：按 `chain` 转发三链，参数透传 |
| `launch/control.launch.py` | mock 链入口：RSP + ros2_control_node + 四 spawner（JSB+JS active / CM inactive / ee_state） |
| `config/<robot>_controllers.yaml` | 按机型控制器配置（500Hz；JS 命令+断流减速 / CM IK 预算 / RT 线程参数），三链共用 |
| `src/supervisor_node.cpp` | 非 RT 诊断：cm status + /joint_states(ERROR) + /ee_state(WARN) 断流 → `/supervisor/alerts` @1Hz，三链均拉起 |
| `scripts/demo_motion.py` | 机型无关演示：关节表/限位解析自 URDF，50Hz JointCommand 点流 |

## 注意事项

- 一次只跑一套栈；换链前 `gz_clean.sh`（单独执行）清残留并等租约。
- `robot:=<机型>` 必填，未知机型显式报错并列出可用项。
- `package.xml` 以 `exec_depend` 登记所有被编排的包，新增记得同步。
