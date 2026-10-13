# unistackbot_bringup

顶层启动包：拉起整机节点、加载控制器配置、编排层诊断。

## 用法

```bash
# 统一入口: chain 一参切三链 (mock|gz|mujoco), 其余参数透传
ros2 launch unistackbot_bringup sim.launch.py chain:=mock robot:=piper

# mock 链（standalone controller_manager + SimControlHardware, 无仿真器）
ros2 launch unistackbot_bringup control.launch.py robot:=piper

# 起链前清场 (三链通用, 单独执行 —— pkill 模式会击杀同命令行宿主, 勿与 launch 同行)
ros2 run unistackbot_bringup chain_clean.sh

# 演示程序住 unistackbot_demo 包 (关节慢流/末端位姿/满速/遥操/点位回放/机型专属)
ros2 run unistackbot_demo demo_motion.py      # 前置: 任一链已启动, JS active
```

gz/mujoco 链的 launch 在 `unistackbot_gazebo` / `unistackbot_mujoco`。

### RViz 可视化（默认不开, 按需 `use_rviz:=true`）

三链默认**不拉 RViz**（自动化路径优先）。桌面调试要看模型动，加参数：

```bash
ros2 launch unistackbot_bringup sim.launch.py chain:=mock robot:=piper use_rviz:=true
ros2 launch unistackbot_bringup control.launch.py robot:=r1_7a use_rviz:=true
```

起的 RViz 接链：TF 树 + RobotModel 实时跟随 /joint_states, 配合 demo/遥操用。
只想摆模型不起控制链 → 用纯可视化链（带关节滑条, 与控制链互斥勿同跑）：

```bash
ros2 launch unistackbot_description display.launch.py \
    model:=$(ros2 pkg prefix --share unistackbot_description)/arms/r1_7a/urdf/r1_7a.urdf.xacro
```

## 可用机型（`robot:=` 参数）

| 机型 | 形态 | 备注 |
|---|---|---|
| `piper` | 6-DoF 球腕臂 + 夹爪 | 参考机型；闭式解析解（analytic_piper） |
| `xarm7` | 7-DoF 臂 | UFACTORY；7 轴业务参考机 |
| `stationlite` | 双臂：底座 + 左右 6-DoF + 夹爪 | Lumos；CM 现只配左臂 |
| `r1_7a` | 宇树七轴冗余臂 | R1-7A；真机 RS485/MIT 接入首目标 |

机型解析 `unistackbot_description/arms/<robot>/` + 本包 `config/<robot>_controllers.yaml`；
未知机型 fail-fast 并列出可用项。各链命令关节集可能有异（如 piper 手指 mock 链直令、
mujoco 链 equality 跟随）——详见 `docs/sim/chain_interface_dictionary.md` §6。

## 关键文件

| 文件 | 说明 |
|---|---|
| `launch/sim.launch.py` | 统一入口：按 `chain` 转发三链，参数透传 |
| `launch/control.launch.py` | mock 链入口：RSP + ros2_control_node + 3 spawner 进程（激活组 JSB+JS+EEB / CM inactive / JTC inactive——JTC 仅 mock 链拉起, MoveIt 执行用） |
| `config/<robot>_controllers.yaml` | 按机型控制器配置（500Hz；JS/CM 点流 + 断流=收敛到最后目标保持 / CM IK 预算 / RT 线程参数），三链共用 |
| `config/<robot>_acceptance.yaml` | 机型验收登记表：形态指纹/容差/mimic 单一事实源，`test/verify_robot.sh` 启动即断言，缺表 fail-fast |
| `src/supervisor_node.cpp` | 非 RT 诊断：cm status（结果码/active 后静默>2s=WARN）+ /joint_states 断流（>1s=ERROR）+ /ee_state 断流（>1s=WARN）→ `/supervisor/alerts` @1Hz，三链均拉起 |
| `scripts/chain_clean.sh` | 三链通用清场：杀残留 launch/节点/gz 家族/挂死 CLI + 停 daemon，exit 0=干净可当门禁 |

## 注意事项

- 一次只跑一套栈；换链前 `chain_clean.sh` 清残留并等租约（**单独执行**）。
- `robot:=<机型>` 必填（可用机型见上表），未知机型显式报错并列出可用项。
- `package.xml` 以 `exec_depend` 登记所有被编排的包，新增记得同步。
