#ifndef UNISTACKBOT_ALGORITHM__IK_SOLVER_HPP_
#define UNISTACKBOT_ALGORITHM__IK_SOLVER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "urdf_fk/urdf_fk.hpp"
#include "unistackbot_interface/ik_result.hpp"
#include "unistackbot_interface/robot_command.hpp"   // RedundancyPreference/RedundancyType

namespace unistackbot_algorithm
{

using unistackbot_interface::IkResult;
using unistackbot_interface::RedundancyPreference;

/*
 * IK 求解器抽象接口 (2026-09-21) —— CM (CartesianMotionController) 的可插拔求解层。
 *
 * 求解器实现契约 (安全相关, 违反 = 控制链事故):
 *
 * 1. **失败绝不改 out_q** —— 无部分解/猜测解混出; 调用方保持上一解是安全语义的根。
 * 2. **timeout 墙钟预算** —— DlsIkConfig::timeout_ns 同款语义: 到时返回 ITERATION_LIMIT
 *    (失败不改输出); RT 周期内调用时预算 = 周期份额 (500Hz → 500µs 量级)。
 * 3. **SolveMode 语义** (粘性场景化解耦, 2026-09-17 定稿):
 *      STREAMING  种子=上一解; 解距种子超跳变阈 = 跳分支, 拒绝输出 (伺服连续性)
 *      COLD_START 种子=分支代表/种子库; 解距种子远是常态不是错误
 * 4. **RT 纪律** —— solve() 跑在控制环上: 零 malloc/锁/printf; 全暂存成员预分配,
 *    实例线程私有 (KDL 有状态暂存, 跨线程并发互踩 —— 见 urdf_fk.hpp 用法契约;
 *    CM 的 update 线程与冷启动 worker 各持一套实例)。
 * 5. **stats 尽力填** —— ok/iterations/solve_us/timed_out 必填; min_sigma 有雅可比
 *    才填 (无则 -1, CM status 通道透传给上层观测)。
 *
 * 接入新求解器 (如用户的 7 轴数值解):
 *   ① 本包新建 <你的求解器>/ 文件夹, 实现 IkSolver (对照 dls_ik/ 为范本)
 *   ② CM on_configure 的选择分支加一行 (yaml 参数 ik_solver: <名字>)
 *   ③ 验证走 docs/ik_validation_playbook.md 六阶段 (Oracle 基准 → 三链回归)
 *   数值类实现与 DlsIk 平级替换/AB 对比; 解析类实现需另议组合策略 (主路径+兜底)。
 */
/*
 * 求解模式 —— 粘性语义的场景化解耦 (2026-09-17 粘性误杀 31/90 冷启动样本后定稿)。
 * (历史位置在 dls_ik.hpp; 2026-09-21 移到接口层 —— 它是求解器通用概念)
 */
enum class SolveMode : uint8_t
{
	STREAMING = 0,
	COLD_START = 1,
};

/*
 * 单次求解统计 (调用方可选消费; ulog/性能画像用)。
 * 名称 DlsIkStats 为历史沿用 (原 DLS 专属, 现为接口级通用统计 —— 数值类实现的
 * restarts/iterations 语义直接适用, 不改名为 IkStats 以免全库 churn)。
 */
struct DlsIkStats
{
	bool ok{false};
	int iterations{0};        // 成功种子的迭代数 (失败=0)
	int restarts_used{0};    // 消耗的重启数 (0 = 调用方种子直接命中)
	double solve_us{0.0};    // 总耗时 [µs]
	double final_err{0.0};   // 失败时的最终误差范数 (成功=0)
	bool timed_out{false};   // 失败是否因 timeout_ns 预算耗尽 (与迭代上限/局部极小区分:
	                        // 超时 ≠ 不可解 —— 放宽预算或换种子仍可解, CM 降级策略据此分流)
	double min_sigma{-1.0};  // 奇异接近度遥测: 最佳配置点的雅可比最小奇异值。
	                        // 成功=解处; 失败=加权误差最低的失败尝试处; 粘性拒绝=被拒解处。
	                        // <0 = 本轮未进入迭代 (NOT_READY/UNREACHABLE 几何预检等)
};

class IkSolver
{
public:
	virtual ~IkSolver() = default;

	// 构造即持有 FK 库 (限位/雅可比来源); 不成功 ready()=false, solve 返回 NOT_READY
	[[nodiscard]] virtual bool init(const UrdfFk * fk, std::string & message) = 0;

	/*
	 * 求解 (点 IK, CM 每控制周期调用)。
	 * seed:   种子关节角 (流式=上一解; 建议传当前 q —— 连续性的根)
	 * red:    冗余偏好 (PRESERVE / LOCK_JOINT; ARM_ANGLE 由实现自决支持与否)
	 * out_q:  成功时写入解; 失败时不修改 (调用方保持上一解) —— 契约 1
	 * stats:  可选出参 (nullptr 忽略) —— 契约 5
	 * mode:   见契约 3; 调用方显式声明意图, 不从种子隐式推断
	 */
	[[nodiscard]] virtual IkResult solve(
		const CartesianPose & target,
		const std::vector<double> & seed,
		const RedundancyPreference & red,
		std::vector<double> & out_q,
		DlsIkStats * stats = nullptr,
		SolveMode mode = SolveMode::STREAMING) const = 0;

	[[nodiscard]] virtual bool ready() const = 0;

	// 种子库 (可选能力, 冷启动加速): 默认不支持 —— 返回 false 非致命,
	// CM worker 已有降级路径 (分支代表+随机阶梯)
	virtual bool loadSeedLibrary(const std::string & path, std::string & message)
	{
		(void)path;
		message = "本求解器未实现种子库 (可选能力)";
		return false;
	}

	[[nodiscard]] virtual std::size_t seedLibrarySize() const
	{
		return 0;
	}
};

}  // namespace unistackbot_algorithm
#endif  // UNISTACKBOT_ALGORITHM__IK_SOLVER_HPP_
