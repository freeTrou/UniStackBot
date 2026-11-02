# 算法域接口与接入

两包：`unistackbot_algorithm`（SDK：接口 + 可选 FK 库，**零 ROS**）＋ `unistackbot_algorithm_solvers`（参考实现，范本用）。
你的包只依赖 SDK 一个，落本仓兄弟目录（或独立 git 仓 clone 进 src/，等价）。

**SDK 不提供**：IK/WB 实现、gravity_torque、collision——算法用什么库自己选。

---

## 1. 接口

### 1.1 IkSolver（臂 IK）

```cpp
// unistackbot_algorithm/ik_solver/ik_solver.hpp
class IkSolver {
public:
    virtual ~IkSolver() = default;
    [[nodiscard]] virtual bool init(const UrdfFk * fk, std::string & message) = 0;
    [[nodiscard]] virtual bool ready() const = 0;
    [[nodiscard]] virtual IkResult solve(
        const CartesianPose & target,        // base 系
        const std::vector<double> & seed,
        const RedundancyPreference & red,    // 7 轴冗余消解（redundancy.hpp）
        std::vector<double> & out_q,
        DlsIkStats * stats = nullptr,
        SolveMode mode = SolveMode::STREAMING) = 0;
    virtual bool loadSeedLibrary(const std::string & path, std::string & message);  // 可选
    virtual void setBudgetMs(double timeout_ms);                                    // 可选
};
```

结果码（`ik_result/ik_result.hpp`）：OK / UNREACHABLE / NEAR_SINGULAR / ITERATION_LIMIT / LIMIT_CONFLICT / NOT_READY / UNSUPPORTED。

**5 条硬契约**：
1. 失败绝不改 out_q
2. 墙钟预算（timeout → ITERATION_LIMIT）
3. SolveMode：STREAMING 连续性优先 / COLD_START 全局搜索
4. 调用域自声明（类注释写明 RT-capable 或回调线程专用）
5. stats 尽力填

### 1.2 WholeBodySolver（全身/多臂）

```cpp
// unistackbot_algorithm/whole_body/whole_body.hpp (v0.3)
class WholeBodySolver {
public:
    virtual ~WholeBodySolver() = default;
    [[nodiscard]] virtual bool init(const WholeBodyConfig & config, std::string & why) = 0;
    [[nodiscard]] virtual WholeBodyResult solve(
        const WholeBodyInput & in,    // 测量 q/dq + goals（按名映射）
        WholeBodyOutput & out) const = 0;   // out.q/dq/ddq/gravity/frames/tasks
};

struct WholeBodyConfig {
    std::string urdf;            // URDF XML 文本（非路径）
    std::string base_frame;
    std::vector<std::string> control_joints;   // 声明序 = 一切数组序
    std::vector<std::string> end_frames;
    std::vector<std::string> telemetry_frames;
    double dt_s;
    std::string vendor_config_path;   // 实现私有参数
};

enum class WholeBodyResult : uint8_t {
    OK = 0, NOT_READY = 1, UNREACHABLE = 2, LIMIT_CONFLICT = 3, SOLVE_FAILED = 4
};
```

**5 条硬契约**：①失败不改 out（先算后写）②同步纯函数（零线程/锁/ROS，温启动住实现内部 mutable）③init 拓扑选拒（joints/frames 与 URDF 不符 = false + why）④诚实拒绝（UNREACHABLE/LIMIT_CONFLICT 是验收面）⑤gravity 无源留空，永不伪造。

**解算域**：壳的 worker 线程按 dt_s 调 solve()——你不知道线程存在。

### 1.3 BaseKinematics（底盘运动学）

`base_kinematics/base_kinematics.hpp`：twist↔轮执行器双向映射 + 注册工厂 `createBaseKinematics(name)`。底盘组用，臂算法组通常不碰。

### 1.4 可用共享库

| 库 | 头 | 内容 |
|---|---|---|
| `UrdfFk` | `urdf_fk/urdf_fk.hpp` | FK、雅可比、限位、链序名（想用就用，不用不碰；实例线程私有） |
| `CartesianPose` | 同上 | xyz + 四元数（m/rad） |

### 1.5 数据边界：给你的 / 你做的 / 不碰的

**摆到你盘子上的**（壳/框架已处理，双臂算法零关心）：

| 数据 | 形态 | 处理者 |
|---|---|---|
| URDF | XML 文本（`WholeBodyConfig.urdf`，非路径） | 壳注入 |
| 测量 | `in.q`/`in.dq` 已按 control_joints 声明序对齐（硬件乱序/名称映射壳全消化） | WB 壳按名映射 state 接口 |
| 目标 | `in.goals` 带帧名（左右目标各一条，`/sdk` 数组→位姿翻译是网关的事） | mos_compat 网关 |
| 节拍 | worker 线程按 dt_s 调 solve()（你不知道线程存在） | WB 壳 |
| 出口安全 | 输出 q 的 NaN 门/限位/步长钳 | WB 壳 |

**还是你的三件事**：

1. **URDF → 你的运动学模型**：init 里自建双臂两条链（`UrdfFk` 现成：`fk.init(urdf, base, tip)` 来两次；用 pinocchio 亦行）
2. **goals 按名映射，不按位置**：goals 是集合，契约不承诺 `goals[0]=左`——按 `goal.frame` 对号（按位置切 = 静默错臂）
3. **输出序 = control_joints 声明序**：init 时从 config 拿到即锁死

一句话：双臂的**数据**（哪些关节/左右哪边/测量来源/结果去向）全不用管；双臂的**几何**（两条链的数学）全是你的。范本边界实物见 `dualarm_dls.hpp`。

---

## 2. 接入（三步）

**第 1 步** 建包：

```bash
cd UniStackBot_ws/src/UniStackBot && mkdir -p their_hqp/src
```

**第 2 步** 四个文件：

package.xml：
```xml
<package format="3">
  <name>their_hqp</name>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>unistackbot_algorithm</depend>
  <depend>pluginlib</depend>
  <depend>Eigen3</depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

CMakeLists.txt：
```cmake
find_package(unistackbot_algorithm REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
add_library(their_hqp SHARED src/hqp_solver.cpp)
target_link_libraries(their_hqp unistackbot_algorithm::unistackbot_algorithm Eigen3::Eigen)
ament_target_dependencies(their_hqp pluginlib)
install(TARGETS their_hqp LIBRARY DESTINATION lib)
pluginlib_export_plugin_description_file(unistackbot_algorithm plugin.xml)
```

plugin.xml：
```xml
<library path="their_hqp">
  <class name="hqp_v1" type="TheirHqpSolver"
         base_class_type="unistackbot_algorithm::WholeBodySolver"/>
</library>
```

src/hqp_solver.cpp：
```cpp
#include "pluginlib/class_list_macros.hpp"
#include "whole_body/whole_body.hpp"

class TheirHqpSolver : public unistackbot_algorithm::WholeBodySolver {
public:
    bool init(const unistackbot_algorithm::WholeBodyConfig & config,
              std::string & why) override {
        // 校验拓扑/读参数；失败: why = 原因, return false
        return true;
    }
    unistackbot_algorithm::WholeBodyResult solve(
        const unistackbot_algorithm::WholeBodyInput & in,
        unistackbot_algorithm::WholeBodyOutput & out) const override {
        // 成功: 填 out 后 return OK；失败: return 码, out 不改
        return unistackbot_algorithm::WholeBodyResult::OK;
    }
};
PLUGINLIB_EXPORT_CLASS(TheirHqpSolver, unistackbot_algorithm::WholeBodySolver)
```

**第 3 步** 构建与选用：

```bash
colcon build --symlink-install --packages-select their_hqp
# 机型 yaml: solver: hqp_v1 —— 完成, 框架包零改动
```

---

## 3. 独立测试（零框架）

```bash
g++ -std=c++17 -O2 -Wall -I<unistackbot_algorithm>/include -I/usr/include/eigen3 \
    test/test_solver.cpp -L<unistackbot_algorithm>/lib -lunistackbot_algorithm \
    -o /tmp/test_solver && /tmp/test_solver
```

## 4. 范本

| 范本 | 位置 | 特点 |
|---|---|---|
| 双臂 DLS（纯仓内栈） | `unistackbot_algorithm_solvers/whole_body/dualarm_dls.hpp` | 零外部库起步 |
| mos WBC 适配层 | 同上 `upper_body_hqp_mos.hpp` + `vendor/mos_wbc/` | 契约换壳不改数学的实例；co-sign 材料在其 README |
| IKFast 适配层 | `unistackbot_algorithm_solvers/analytic_piper/` | 闭式解接入样例 |

> whole_body 契约 v0.3 待算法组 co-sign（行为变更对照表：`unistackbot_algorithm_solvers/whole_body/README.md`）。
