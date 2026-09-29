# stationlite MuJoCo 资产 (双臂机型, 2026-09-29 调通)

`stationlite.xml` — Lumos Station Lite 双臂 (底座 + 左右对称 6-DoF + 平移夹爪) 的
MJCF 模型。**与 piper 不同: 手写策展**, 未走 `robot_description_to_mjcf` 转换器
(双臂树/URDF 结构简单, 手写更快)——也因此踩了转换器产物天然免疫的坑, 全部记录在
下方调试档案, 下一台手写机型的必读。

## 文件

| 文件 | 作用 |
|---|---|
| `stationlite.xml` | 单文件 MJCF: 身体树/惯量/限位照抄 URDF, visual mesh 直引 `../meshes/visual/` STL (零复制) |

## 模型约定 (对齐 mujoco_ros2_control 0.1.2)

- position 执行器 name == joint name (插件按名映射); 14 个执行器 = 左右各 6 主关节 + 1 夹爪主指
- **手指 joint2 无 joint 声明** (body weld 死): equality 耦合暂退役 (调试期简化, 抓取仿真
  启用时按 piper 的 equality 模式补回, 注意方向坑见 piper README)
- URDF ros2_control 块 MuJoCo 模式下手指 joint2 无 state 声明, 与 MJCF 一致
- timestep 0.002 (500Hz) 与控制器 update_rate 对齐
- 关节全部带 `damping/frictionloss/actuatorfrcrange` (力矩界 = URDF effort: j1-3=27,
  j4=7, j5-6=12.5, 夹爪=150) —— 见调修档案 #1, 裸关节是发散的
- 碰撞全禁用 (floor 除外): 调通期简化; 真开碰撞时按 mesh 碰撞重做 (piper 模式),
  勿恢复占位胶囊 (见 #3)

## 调修档案 (2026-09-29, 按破案顺序)

### #1 QACC 发散 → 154 万条 Linesearch + 激活超时

- **症状**: MuJoCo 窗口/日志被 `Linesearch objective is not convex` 刷屏 (154 万条),
  spawner `load_controller` 3×10s 超时全灭。demo "50Hz 达成" 是假绿——纯发布 topic
  不需要订阅者。
- **误诊弯路**: mesh 碰撞 (早已 contype=0)、equality (删了照旧)、关节压限位线
  (挪开照旧)——三个约束源逐一排除。
- **破案**: 离线 `mj_step` 第一帧即 `QACC NaN at DOF 6, Time=0.0000`——不是约束,
  是**数值发散**; Linesearch 洪流→stderr IO 拖死物理线程→激活超时, 全是下游症状。
- **根因**: 关节**零阻尼** + 高 kp 位置执行器 + 小惯量 (手指 8e-6)。对照 piper.xml
  一眼定位: piper 每个 joint 都有 `damping/frictionloss/actuatorfrcrange`, 本模型裸。
- **修复**: 全部关节补三属性 (值见上)。验收: 离线 3000 步零警告, 链上 4 控制器就位。
- **教训**: **手写 MJCF 前先 diff 已验证模型的同部位属性**; "Linesearch 洪流"第一
  反应应是查 QACC NaN (发散), 而非约束问题。

### #2 mesh 渲染成"球串" (RViz 正确, MuJoCo 完全不对)

- **症状**: MuJoCo 窗口显示一堆大小球组成的链, RViz 同一批 URDF/mesh 正常。
- **破案链**: 打印 `geom_type` → 全部 `mjGEOM_SPHERE`; `mj_saveLastXML` 回显发现
  `mesh=` 属性被替换成等效惯量球 (size=包围球半径, pos=mesh 质心)——**mesh 顶点
  数据其实加载正常** (惯量全对, 所以 #1 修复后链路能通, 只是渲染错)。
- **根因**: default 类模板缺 `type="mesh"`。MuJoCo 3.x 里 `mesh=` 属性**不隐式设定
  geom 类型**, geom 落到默认 `SPHERE`。piper.xml 由转换器生成, 其 default 类自带
  `type="mesh"` 天然免疫。
- **修复**: `<default class="vis"><geom type="mesh" .../></default>`。
- **诊断弯路 (值得记)**: pip mujoco 3.8.1 + `from_xml_string` 的最小复现里连 piper
  的 STL 都变球 (假阳性差点引向"STL 文件问题"); 装 apt 同版 3.12 venv + 完整文件
  `from_xml_path` 对照才把问题域钉死。**诊断工具自身要交叉验证; 最小复现不还原
  现场时, 以完整现场为准**。
- **教训**: 用户侧关键证据 ("RViz 对、MuJoCo 是球") 一步排除一半假设空间——mesh
  文件无罪, 只查 MJCF。

### #3 夹爪前端胶囊穿帮

- **症状**: 显示修好后手指处仍有胶囊鼓出。
- **根因**: 调通期占位胶囊 (`*_col`, 半径 0.03) 统一规格; 手指 mesh 薄小 (3.6cm),
  胶囊比 mesh 本身大, 穿出露馅。臂粗处的胶囊藏在 mesh 里看不见。
- **修复**: 删全部 19 个占位胶囊。**安全性账**: contype=0 禁碰撞后零物理作用;
  惯量来自显式 `<inertial>` 不受 geom 增删影响; 离线+链上回归全绿。
- **教训**: 占位几何要么做对尺寸要么删掉——"看不见"依赖巧合 (藏在大 mesh 里),
  换小部件就穿帮。

### #4 换模型后链卡死 (伪装成模型问题)

- **症状**: 删胶囊 rebuild 后 `load_controller` 无响应、`list_controllers` 超时。
- **根因**: **不是模型**——上一条 GUI 链未彻底清场, 新旧同名节点打架。干净复现
  (chain_clean → 重起) 4 控制器全就位。
- **教训**: 调模型期间反复起链, **每次 rebuild/换模型前先 chain_clean 且单独执行**;
  链路卡死类症状先排除残留再怀疑刚改的东西。

### #5 伺服振动 ("是模拟真实电机吗"——不是)

- **症状**: 显示/发散修后臂持续嗡振, 静止段也振。
- **数据**: 录包谱分析, 主频全钉奈奎斯特 (≥500Hz 采样率折叠) → 真实频率在伺服
  步率附近, 机械共振不可能在此 (真机模态 10-100Hz); j6 速度 std 18.6 rad/s。
- **根因**: `ω_n=√(kp/I)·dt` 击穿离散稳定边界 (j6: 200/4.3e-4 → **1.36 > 1**) →
  位置伺服极限环。MuJoCo 执行器是理想伺服 (τ=kp·e−kv·q̇), 无齿槽/摩擦极限环/
  编码器量化——真机静止是"真死", 仿真振了就是增益失配。
- **修复**: 末梢小惯量关节 kp 降档 (j4 2000→1000, j5 300→100, j6 200→80, 夹爪
  200→120) + j5/j6 关节 damping 0.1→0.3。主关节 j2/j3 不动 (整臂展开惯量
  ~0.65 kg·m², ω_n·dt=0.19 稳定域内; kp 6000 下重力下垂仅 0.06 mrad)。
- **验收**: 链上静止段 14 关节位置 std 全 0 (修前 j6=0.019 rad ≈1.1°)。
- **已知残留 (接受)**: j2/j4 ±0.03°/0.04° @500Hz 微颤, 末端等效 ~0.5-0.7mm;
  GUI 逐帧渲染放大可见, `/joint_states` 470Hz 采样欠奈奎斯特显示为 0。成因是
  dampratio 阻尼按零位构型线性化, 运动中阻尼比偏离 1.0。判定为固有底噪: mujoco
  链定位是动力学回归, 亚毫米精度评估走 mock 链。若需再压: timestep 0.001
  (微颤 ~×¼, CPU ×2) 或 j2/j4 继续降 kp (下垂账尚有余量)。

### #6 运动中连杆"完全分离" (静态却完美) —— 两层错叠加

- **症状**: 零位静态显示完美; 一跑 demo, 连杆整段甩离正确位置, 关节与臂脱开。
- **两层根因** (先后修掉):
  1. `joint pos` 写了 URDF origin 平移量 (同一向量与 body pos 用两次)。URDF 轴
     过 child 原点, joint pos 恒 0——piper.xml (转换器产物) 全部 pos="0 0 0" 为证。
  2. **joint 挂在 parent body 里** (真凶): MJCF 语义 = body 内声明的 joint 旋转
     **该 body 自己**; URDF 的 joint i 属于 child link i。挂错一层后 j2 转动变成
     "link1 壳绕 j1 轴心甩" 而 armbase 壳不动 → 两壳脱开。零位时相邻两层原点
     重合 (link1 pos=0) → 静态完美, 一动才暴露。
- **修复**: joint pos 全部 0 + 每个 joint 移入其 child body 开标签之后。
- **验证 (防复发的硬方法)**: 与 MuJoCo 直载原始 URDF 的模型做**同 qpos 全 body
  世界坐标对照**——零位/j2/j2+j3/j1+j2+j3+j4 四组全部偏差 0.00000000 m。
- **教训**:
  - "相邻 body 原点距离" 是树结构数学恒等式, 任何挂法都成立, **抓不住挂载错**;
    必须与 URDF 基准做全 body FK 对照。
  - 静态零位对 ≠ 模型对 (零位是多层错误的共同盲区); 机型 MJCF 验收必须含
    **多关节非零 qpos 的 FK 数值对照 + 运动中帧检查**。
  - MuJoCo URDF 导入器会 weld 合并 fixed 链上的无关节 body (base/armbase 被并进
    world), 对照脚本注意 body 缺失 (name2id 返回 -1, 负索引坑)。

## 验证状态 (2026-09-29)

- 离线: 3000-4000 步零警告, 零目标静止收敛
- 链上: 4 控制器就位 (3 active + CM inactive), demo_motion 50Hz 达成,
  静止段 14 关节位置 std=0, GUI 显示与 RViz 一致
- 提交: bff9b3e (发散) → 0cbbef6 (type=mesh) → dcd9d77 (删胶囊) → 7d96fa9 (kp 调参)
