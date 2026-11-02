# 算法域架构定稿（2026-11-01，同日原则性修正）

> **性质**：normative——本文件是算法域包结构的**唯一权威定义**，与实现冲突时以本文件为准。
> **依据**：MoveIt / ros2_control / Nav2 三个头部生态的插件架构先例；六机型实证；mos 合并策略（ODE）。
> **决策者**：用户，2026-11-01 终审。
> **修正**（2026-11-01 同日）：SDK 不提供"库的等价替代品"——算法需要什么用什么，SDK 只定义接口不选库。
> **形态定稿**（2026-11-02）：**"外部仓" = 同仓外部化**——外部团队的包以兄弟包形态住本仓（StationPro 分支），物理不拆分；边界靠归属/依赖单向/框架零改动三纪律维持。物理独立 git 仓仍被 pluginlib 路径支持，但不再是目标形态。

---

## 0. 核心原则（本次修正新增）

**SDK 定义接口，不选库。**

- 算法团队在实现里用 pinocchio / KDL / qpoase / OSQP / 自研，都是他们自己的选择
- SDK 的 `urdf_fk` 是**可选便利**——想用就用，不想用完全不碰
- SDK **不提供** gravity_torque / collision 等"等价替代品"
- 重力补偿、碰撞检测等算法的归宿 = 算法团队自己的外部包（同仓外部化，见形态定稿）

---

## 1. 定稿方案：两包 + 外部包（同仓外部化）

```
┌─────────────────────────────────────────────────────────────┐
│ Package 1: unistackbot_algorithm                            │
│ = 算法 SDK（契约 + 可选共享 FK + 底盘运动学）                │
│                                                             │
│  维护者: 框架组（API 稳定性）+ 算法组（urdf_fk 改进）        │
│  变更频率: 低（契约变更 = 破坏性变更，需评审）               │
│                                                             │
│  契约头（算法团队的 API 面，纯头文件）:                      │
│  ├── ik_solver/ik_solver.hpp       ← IkSolver 接口           │
│  ├── whole_body/whole_body.hpp    ← WholeBodySolver 接口     │
│  └── base_kinematics/base_kinematics.hpp ← BaseKinematics   │
│                                                             │
│  可选共享库（想用就用，不用不碰）:                            │
│  └── urdf_fk/urdf_fk.cpp           ← FK/雅可比              │
│                                                             │
│  底盘运动学（BaseVelocityController 编译期消费）:            │
│  ├── base_kinematics/base_kinematics.cpp ← Diff             │
│  └── base_kinematics/n_steer_kinematics.cpp ← NSteer       │
│                                                             │
│  构建: libunistackbot_algorithm.so                          │
│  依赖: Eigen + KDL + urdfdom（零 ROS 零 nlopt 零 rclcpp）  │
│  ARM 交叉编译: ✓（总线主站够用）                            │
│                                                             │
│  ⚠️ 不含: gravity_torque / collision / 任何算法实现         │
│     —— 算法团队自己管，SDK 不替他们选库                     │
└─────────────────┬───────────────────────────────────────────┘
                  │ depend (编译期链接 SDK 的 .so)
                  ▼
┌─────────────────────────────────────────────────────────────┐
│ Package 2: unistackbot_algorithm_solvers                    │
│ = 参考实现 + pluginlib 注册（臂算法组维护）                  │
│                                                             │
│  维护者: 臂算法组（CODEOWNERS）                              │
│  变更频率: 高（随时加实现，不影响 SDK）                      │
│                                                             │
│  IK 实现:                                                   │
│  ├── dls_ik/  (StationPro 分支出清, 住 dev/main) ← DLS 数值 IK │
│  ├── analytic_piper/ + IKFast vendored ← piper 闭式解      │
│  ├── analytic_stationlite/ + IKFast vendored ← stationlite │
│  ├── analytic_common/              ← 解析解公共件           │
│  └── trac_ik_solver/ + nlopt vendored (可选编译)           │
│                                                             │
│  WholeBody 实现:                                            │
│  ├── whole_body/dualarm_dls.hpp    ← 双臂占位实现           │
│  ├── whole_body/single_arm_dls.hpp (StationPro 分支出清)     │
│  └── whole_body/upper_body_hqp_mos.hpp + vendor ← mos 适配 │
│                                                             │
│  Plugin 注册:                                               │
│  ├── ik_plugins/ik_plugins.cpp + xml     ← IK 插件注册     │
│  ├── ik_plugins/ik_plugins_trac_ik.cpp + xml                │
│  └── ik_plugins/whole_body_plugins.cpp + xml ← WB 注册     │
│                                                             │
│  构建: libik_solvers.so + libwhole_body_solvers.so          │
│  依赖: Package 1 + pluginlib + rclcpp(trac_ik) + nlopt    │
│  ARM 交叉编译: 不需要                                       │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ 外部包 × N（同仓外部化 = 最终形态, 2026-11-02 定稿）        │
│                                                             │
│  物理形态: 本仓兄弟包（StationPro 分支）, 不拆独立 git 仓   │
│  逻辑形态: 外部仓——归属外部维护者, 框架组当租户对待        │
│  依赖: 只依赖 Package 1（SDK）+ 自选库                      │
│  注册: 自带 plugin.xml，pluginlib 运行期自动发现            │
│  我们的框架包: 零改动                                       │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 为什么是这个方案（三个先例验证）

| 生态 | SDK（契约） | 参考实现 | 外部插件 |
|---|---|---|---|
| **MoveIt** | `moveit_core`（kinematics_base.hpp） | `moveit_kinematics`（KDL/LMA/SRV） | `trac_ik_kinematics_plugin`（独立仓） |
| **ros2_control** | `hardware_interface`（SystemInterface） | `mock_components` | 我们（UniStackBot）的 SimControlHardware |
| **Nav2** | `nav2_core`（Controller/Planner 接口） | `dwb_controller` | 各大学规划器仓 |
| **UniStackBot** | `unistackbot_algorithm`（IkSolver/WB/BaseKin） | `unistackbot_algorithm_solvers` | 算法团队的 HQP 包（同仓外部化） |

**共同模式**：契约 SDK 在框架主仓（稳定、极少变）→ 参考实现在框架仓内（活文档）→ 外部实现在独立包（同仓外部化=团队自主，物理独立仓亦可）→ pluginlib 运行期发现 → yaml 一行选用。

**SDK 自包含（2026-11-02 收口）**：IkResult（`ik_result/ik_result.hpp`）与 RedundancyPreference/RedundancyType（`ik_solver/redundancy.hpp`）已从 interface 迁入 SDK——算法契约词汇归契约的主人。**SDK 零本仓包依赖**（ldd 实证：libunistackbot_algorithm.so 不链任何 unistackbot 库），**算法团队最小闭包 = `unistackbot_algorithm` 一个包**。运行面反向引用：interface 的 RobotCommand.redundancy 字段经 include 引用算法域类型（方向 = 运行面引算法层，与 controller 同款，无环；硬件四子包零 ROS 链不受影响）。

**否决的替代方案**：
- ❌ SDK 独立成仓：跨仓版本同步地狱；参考实现与契约脱节；非行业标准
- ❌ 三包拆分（SDK + 臂 + 底盘）：BaseVelocityController 跨包链接工厂断裂
- ❌ 单包不拆：nlopt/rclcpp 泄入核心库，方案 C 纯度被打破

---

## 3. 接口文档（算法团队接入指南）

### 3.1 你要实现什么

| 算法类型 | 契约接口 | 头文件 | 你要实现的方法 |
|---|---|---|---|
| 臂 IK | `IkSolver` | `ik_solver/ik_solver.hpp` | `init()` + `solve()` |
| 全身/多臂 | `WholeBodySolver` | `whole_body/whole_body.hpp` | `init()` + `solve()` |
| 底盘运动学 | `BaseKinematics` | `base_kinematics/base_kinematics.hpp` | `init()` + 逆解/正解 |
| 碰撞检测 | 无独立契约* | —（算法团队自带） | — |
| 重力补偿 | 无独立契约* | —（算法团队自带） | — |

> *重力/碰撞作为 `WholeBodyOutput.gravity` / `WholeBodyOutput.tasks` 字段由求解器实现填写，SDK 不提供独立契约（2026-11-01 修正：算法用什么库自己选）。

### 3.2 IkSolver 接口（臂 IK）

```cpp
// 文件: unistackbot_algorithm/ik_solver/ik_solver.hpp
class IkSolver {
public:
    virtual ~IkSolver() = default;

    // 装配期: 初始化（传入 FK 库, 指纹选拒不代选）
    [[nodiscard]] virtual bool init(const UrdfFk * fk, std::string & message) = 0;
    [[nodiscard]] virtual bool ready() const = 0;

    // 每次求解: 点 IK（CM 在回调线程调用, 不在 RT 环）
    [[nodiscard]] virtual IkResult solve(
        const CartesianPose & target,        // 目标位姿（base 系）
        const std::vector<double> & seed,    // 种子关节角
        const RedundancyPreference & red,    // 冗余偏好
        std::vector<double> & out_q,         // 输出解（失败时不改）
        DlsIkStats * stats = nullptr,        // 可选统计
        SolveMode mode = SolveMode::STREAMING) = 0;

    // 可选能力（默认不支持, 返回 false）
    virtual bool loadSeedLibrary(const std::string & path, std::string & message);
    virtual void setBudgetMs(double timeout_ms);
};
```

**实现契约（5 条硬规则）**：
1. **失败绝不改 out_q**——无部分解混出
2. **墙钟预算**——timeout 到时返回 ITERATION_LIMIT
3. **SolveMode 语义**——STREAMING（连续性优先）/ COLD_START（全局搜索）
4. **调用域自声明**——RT-capable（DlsIk 类）或回调线程专用（TracIk 类）
5. **stats 尽力填**——ok/iterations/solve_us/min_sigma

### 3.3 WholeBodySolver 接口（全身/多臂）

```cpp
// 文件: unistackbot_algorithm/whole_body/whole_body.hpp (v0.3)
class WholeBodySolver {
public:
    virtual ~WholeBodySolver() = default;

    // 装配期: 初始化（拓扑选拒, 不代选）
    [[nodiscard]] virtual bool init(const WholeBodyConfig & config, std::string & why) = 0;

    // 每次解算: 同步纯函数（壳的 worker 线程调用）
    [[nodiscard]] virtual WholeBodyResult solve(
        const WholeBodyInput & in,    // 测量 + 目标
        WholeBodyOutput & out          // 解（失败不改）
    ) const = 0;
};

// 配置: 全部显式无默认
struct WholeBodyConfig {
    std::string urdf;                       // URDF XML 文本（非路径！）
    std::string base_frame;                 // 目标参考系
    std::vector<std::string> control_joints; // 解算器优化的关节集
    std::vector<std::string> end_frames;     // 末端帧集合
    std::vector<std::string> telemetry_frames; // 遥测帧表
    double dt_s;                            // worker 节拍约定值
    std::string vendor_config_path;         // 实现私有参数
};

// 结果: 五值枚举（诚实拒绝）
enum class WholeBodyResult : uint8_t {
    OK = 0, NOT_READY = 1, UNREACHABLE = 2,
    LIMIT_CONFLICT = 3, SOLVE_FAILED = 4
};
```

**实现契约（5 条硬规则）**：
1. **失败绝不改 out**——先算后写
2. **同步纯函数**——零线程/零锁/零 ROS；温启动状态住实现内部（mutable）
3. **init 指纹选拒**——joints/frames 与 URDF 不符 = false + why
4. **诚实拒绝进 TaskDiag**——UNREACHABLE/LIMIT_CONFLICT 是验收面
5. **gravity 可留空**——运动学栈无动力学源时诚实留空（永不伪造）

**解算域**：worker（壳起线程按 dt_s 节拍调 solve()，求解器不知道线程存在）。

### 3.4 BaseKinematics 接口（底盘运动学）

```cpp
// 文件: unistackbot_algorithm/base_kinematics/base_kinematics.hpp
class BaseKinematics {
public:
    virtual ~BaseKinematics() = default;
    // 详见头文件——接口含 twist↔轮执行器映射 + 逆解/正解
    // 已有实现: diff（差速）/ n_steer（N 舵轮参数化）
    // 未来: 4ws（mos M3 重述）
};
```

### 3.5 你能用的共享库

| 库 | 头文件 | 用途 |
|---|---|---|
| `UrdfFk` | `urdf_fk/urdf_fk.hpp` | FK（关节角→末端位姿）、雅可比、限位、链序名 |
| `CartesianPose` | `urdf_fk/urdf_fk.hpp` | 位姿载体（xyz + 四元数，单位 m/rad） |
| `SolveMode` | `ik_solver/ik_solver.hpp` | 流式/冷启动枚举 |
| `WholeBodyResult` | `whole_body/whole_body.hpp` | 五值结果码 |
| `TaskState` | `whole_body/whole_body.hpp` | 四值任务状态 |

---

## 4. 外部包接入操作（三步完成）

> 物理落点两可（2026-11-02 形态定稿）：**首选 = 本仓兄弟包**（同仓外部化最终形态，目录即归属边界）；
> 亦支持物理独立 git 仓 clone 进工作区（colcon 视角等价）。下文以同仓包为默认路径。

### 第 1 步：建你的包目录

```bash
cd UniStackBot_ws/src/UniStackBot        # 本仓根
mkdir -p their_hqp/src && cd their_hqp   # 兄弟包目录 = 归属边界
```

### 第 2 步：写这四个文件

**package.xml**:
```xml
<?xml version="1.0"?>
<package format="3">
  <name>their_hqp</name>
  <version>0.1.0</version>
  <description>Algorithm team's HQP solver</description>
  <maintainer email="algo@team.com">Algorithm Team</maintainer>
  <license>Proprietary</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>unistackbot_algorithm</depend>
  <depend>pluginlib</depend>
  <depend>Eigen3</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

**CMakeLists.txt**:
```cmake
cmake_minimum_required(VERSION 3.8)
project(their_hqp)
find_package(ament_cmake REQUIRED)
find_package(unistackbot_algorithm REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(their_hqp SHARED src/hqp_solver.cpp)
target_compile_features(their_hqp PUBLIC cxx_std_17)
target_link_libraries(their_hqp
    unistackbot_algorithm::unistackbot_algorithm Eigen3::Eigen)
ament_target_dependencies(their_hqp pluginlib)

install(TARGETS their_hqp LIBRARY DESTINATION lib)
pluginlib_export_plugin_description_file(unistackbot_algorithm plugin.xml)
ament_package()
```

**plugin.xml**:
```xml
<library path="their_hqp">
  <class name="hqp_v1"
         type="TheirHqpSolver"
         base_class_type="unistackbot_algorithm::WholeBodySolver">
    <description>Algorithm team's HQP solver v1</description>
  </class>
</library>
```

**src/hqp_solver.cpp**:
```cpp
#include "pluginlib/class_list_macros.hpp"
#include "whole_body/whole_body.hpp"
#include "urdf_fk/urdf_fk.hpp"

class TheirHqpSolver : public unistackbot_algorithm::WholeBodySolver {
public:
    bool init(const unistackbot_algorithm::WholeBodyConfig & config,
              std::string & why) override {
        // 初始化你的求解器（读 URDF、配置参数、校验拓扑）
        // 失败时: why = 原因; return false
        return true;
    }

    unistackbot_algorithm::WholeBodyResult solve(
        const unistackbot_algorithm::WholeBodyInput & in,
        unistackbot_algorithm::WholeBodyOutput & out) const override {
        // 你的 HQP 解算
        // 成功: 填 out.q/dq/ddq/gravity/frames/tasks; return WholeBodyResult::OK
        // 不可达: return WholeBodyResult::UNREACHABLE (out 不改!)
        // 限位冲突: return WholeBodyResult::LIMIT_CONFLICT (out 不改!)
        return unistackbot_algorithm::WholeBodyResult::OK;
    }
};

PLUGINLIB_EXPORT_CLASS(TheirHqpSolver, unistackbot_algorithm::WholeBodySolver)
```

### 第 3 步：构建集成

```bash
cd UniStackBot_ws
colcon build --symlink-install --packages-select their_hqp
# yaml 改一行:
# whole_body_controller.ros__parameters.solver: hqp_v1
# 完成——框架包零改动
```

（物理独立仓路径：`git clone` 进 `src/` 后同命令——colcon 视角两者等价，pluginlib 发现机制相同。）

### 独立测试（不需要框架）

```bash
# g++ 直编，零 ROS 依赖
g++ -std=c++17 -O2 -Wall \
    -I<unistackbot_algorithm>/include \
    -I/usr/include/eigen3 \
    test/test_solver.cpp \
    -L<unistackbot_algorithm>/lib -lunistackbot_algorithm \
    -o /tmp/test_solver && /tmp/test_solver
```

---

## 5. Plugin 注册表（已注册的可选实现）

| 插件名 | 类型 | 包 | 描述 |
|---|---|---|---|
| `dls` | IkSolver | algorithm_solvers | DLS 数值 IK（RT-capable） |
| `analytic_piper` | IkSolver | algorithm_solvers | piper 球腕闭式解 |
| `analytic_stationlite` | IkSolver | algorithm_solvers | stationlite 平行轴闭式解 |
| `trac_ik` | IkSolver | algorithm_solvers | TRAC-IK 数值（回调线程专用） |
| `dualarm_dls` | WholeBodySolver | algorithm_solvers | 双臂独立链 DLS 占位 |
| `hqp_v1` | WholeBodySolver | **their_hqp**（外部包，同仓兄弟目录） | 算法团队的 HQP |

---

## 6. 各算法域归属终表（2026-11-01 修正：SDK 不做库的替代品）

| 算法域 | 契约在 | 参考实现在 | 外部包 | 维护者 | SDK 提供？ |
|---|---|---|---|---|---|
| 臂 IK | Pkg 1 | Pkg 2 | 可有 | 臂算法组 | 契约 + 可选 urdf_fk |
| 全身/多臂 | Pkg 1 | Pkg 2 | **主力** | 臂算法组 | 契约 + 可选 urdf_fk |
| 底盘运动学 | Pkg 1 | Pkg 1（.cpp） | 可有 | 底盘算法组 | 契约 + 实现（控制器需要） |
| **重力补偿** | **无独立契约** | — | **算法团队自带** | **臂算法组** | **✗ 不提供**（算法用什么库自己选） |
| **碰撞检测** | **无独立契约** | — | **算法团队自带** | **臂算法组** | **✗ 不提供**（算法用什么库自己选） |
| FK 共享 | — | — | — | — | 可选便利（urdf_fk，想用就用） |

**修正说明**（2026-11-01 同日）：之前把重力补偿和碰撞检测放在 Pkg 1 是错误的——那等于 SDK 替算法团队选了库（KDL 替 pinocchio）。正确做法：算法团队在自己仓里想用什么就用什么，SDK 只提供接口和可选的 FK 便利。重力/碰撞作为 `WholeBodyOutput` 的字段（`gravity`/`tasks`）由求解器实现填写，SDK 不需要独立契约。

---

## 7. 头部/腰部自由度处理

| 部件 | 算法域？ | 归宿 | 说明 |
|---|---|---|---|
| 头部 (2 DOF) | ✗ | 控制器域 (js_head) | 纯关节空间，零算法 |
| 腰部/升降 (被动) | ✓ FK 链中 | Pkg 1 (base_frame 选择) | 测量参数，不优化 |
| 腰部/升降 (主动) | ✓ 控制集中 | Pkg 2 或外部仓 | 未来：HQP 优化 15-DOF |
| 底盘轮系 (8 DOF) | ✓ | Pkg 1 (base_kinematics) | 底盘组维护 |

---

## 8. CODEOWNERS（团队权限分离）

```gitignore
# unistackbot_algorithm/CODEOWNERS
# 契约头 = 框架组（API 变更需评审）
/ik_solver/           @框架组
/whole_body/whole_body.hpp  @框架组
/base_kinematics/base_kinematics.hpp  @框架组 @底盘组

# 实现目录 = 各算法组（独立工作）
/urdf_fk/             @臂算法组
/base_kinematics/*.cpp @底盘算法组

# unistackbot_algorithm_solvers/CODEOWNERS
# 全部 = 臂算法组
*  @臂算法组
```
