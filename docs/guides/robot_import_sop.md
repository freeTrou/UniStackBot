# 机型接入 SOP：URDF 导入 → 三链仿真可视化（2026-10-14 立）

> 定位：**从拿到一个 URDF 文件到三链（mock/gz/mujoco）可视化全绿**的单页标准流程。
> 本文档是流程正文——坑的完整档案在各步引用的 README 里，不需要预先读完。
> 六次导入实战（piper/xarm7/stationlite 五轮修/r1_7a 一次过/r1_pro+galbot_s1 留档）的经验全部索引化于此。

## 流程总览（S0→S6 线性，每步有门，门不过即停）

```
S0 原始档案 → S1 结构解剖 → S2 xacro 四件套 → S3 静态目检 → S4 mock 链 → S5 mujoco 链 → S6 gz 链
   死档案       解剖器报告     插件断言③½      display RViz    verify 静态+JS/CM  MJCF+verify      ign 分支+verify
```

## S0 原始档案（死档案）

**做什么**：`<category>/<robot>/original/` 存上游原样快照——URDF + meshes + LICENSE + README
（来源 URL / 提取提交 / License 三要素）。

**门**：登记表更新（`unistackbot_description/README.md` 机型登记表——**接入必登记**，开源合规单一入口）。

**坑速查**：方法论文档 `arms/original_README.md`；分类目录 arms / mobile_manipulators / humanoids。

## S1 结构解剖

**做什么**：`ros2 run unistackbot_tool robot_dissect.py <robot>.urdf`

**输出**：控制域分类（轮/腰/臂/夹爪/传感器挂点）+ 类型分布 + 链拓扑（CM base→tip 候选）
+ **预警**（continuous 轮→velocity 接口域 / prismatic→单位米 / 容量核算 >16 分实例）
+ acceptance.yaml 指纹登记草稿。

**门**：报告里"未归类"必须为零（名称语义不识别的关节先人工归类再重跑）；
四件式分割预案在此定（臂实例 ≤16 / 腰独立 / 轮走生态件）。

## S2 xacro 四件套（照 piper 模板抄，数字零手抄）

| 文件 | 要点 | 坑 |
|---|---|---|
| `<robot>.urdf.xacro` | 顶层入口五参数：use_gripper / use_ros2_control / use_world / use_gazebo / headless | — |
| `<robot>_macro.xacro` | 运动学宏；use_world 时加 world 并固定（**轮式机型改 planar/free 连接**——新维度） | — |
| `<robot>_ros2_control.xacro` | `<ros2_control>` 块；use_gazebo 四态插件分支 | **ign 分支三件缺一不可**：GazeboSimSystem + `<gazebo>` 块（gz_ros2_control-system + yaml 路径）+ 限位线关节 `initial_value`（gz 咬死坑）。r1_7a 漏前两件假绿 17 天（2026-10-13 破案） |
| 夹爪宏（如有） | prismatic 主关节 + mimic 手指 | 空 link 也必须留 `<inertial>`（Gazebo 丢无惯量 link） |

配套：bringup `config/<robot>_controllers.yaml`（四控制器 + ik_solver 必填 + trac_ik 机型
**必须加 `ik_timeout_mode: message_rate`**——2026-10-12 卡顿根因）。

## S3 静态目检

**做什么**：`ros2 launch unistackbot_description display.launch.py model:=<xacro 路径>`——RViz 里
模型完整、关节拖动正常、mesh 无缺失。

**门**：人眼无异常（断裂/穿模/丢 link）。

## S4 mock 链（第一态）

**做什么**：
```bash
ros2 run unistackbot_bringup chain_clean.sh        # 单独执行！
bash test/verify_robot.sh <robot>                  # 静态段 + JS 流 + CM 切换
```
前置：`config/<robot>_acceptance.yaml` 建表（指纹登记值从 S1 报告 + 展开实测，勿手抄）。

**门**：verify 全绿——含**判据③½插件断言**（per-chain 硬件插件名 = 登记值，2026-10-14 立）。

## S5 mujoco 链（第二态）

**做什么**：MJCF 生成（`robot_description_to_mjcf` + 手工策展；**有官方 MJCF 的机型从官方起点策展**，
如 galbot_s1）→ `bash test/verify_robot.sh <robot> --with-mujoco`。

**门**：verify mujoco 段全绿（JS 跟踪 + CM 收敛进容差）。

**坑速查**：七坑纪律全部在 `arms/stationlite/mujoco/README.md` 调修档案（发散/球串渲染/胶囊穿帮/
链残留卡死/伺服振动/关节挂 child body/equality 方向——**手写 MJCF 前必读**）；piper 的
equality 方向坑在 `arms/piper/mujoco/README.md`。

## S6 gz 链（第三态）

**做什么**：S2 的 ign 分支就位后 `bash test/verify_robot.sh <robot> --with-gazebo`。

**门**：verify gz 段全绿（跟踪 + 桥语义 + pause/resume）。

**坑速查**：咬死坑与冻结坑在 `unistackbot_description/README.md` 注意事项；gz 链起链前
`gz_clean.sh` 深清 + daemon stop + 等租约（chain_clean 不够时）。

## 可视化收尾（三态都看一眼）

```bash
# 每链起后 RViz（或 GUI 链窗口）确认运动正常; display 链 = 第 0 态纯可视化
ros2 launch unistackbot_bringup sim.launch.py chain:=mock robot:=<robot> use_rviz:=true
```

## 流程外的已知纪律

- 一次只跑一套栈；换链必 chain_clean（**单独执行**，勿与 launch 同命令行）。
- 机型/拓扑/算法选择参数无默认值（robot/model/ik_solver 必填 fail-fast）。
- 新机型接入后跑完 S4-S6 = pipeline A-D 阶段过半；E(RT 基准)/F(数据场景)/G(归档)
  见 `docs/sim/sim_validation_pipeline.md`（完整 A-G 门禁）。
