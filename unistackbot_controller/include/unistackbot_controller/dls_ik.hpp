#ifndef UNISTACKBOT_CONTROLLER__DLS_IK_HPP_
#define UNISTACKBOT_CONTROLLER__DLS_IK_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "unistackbot_controller/urdf_fk.hpp"
#include "unistackbot_interface/ik_result.hpp"
#include "unistackbot_interface/robot_command.hpp"   // RedundancyPreference/RedundancyType

namespace unistackbot_controller
{

// 契约类型来自 interface 包 (跨层共享); 本命名空间内直接使用
using unistackbot_interface::IkResult;
using unistackbot_interface::RedundancyPreference;
using unistackbot_interface::RedundancyType;

/*
 * 数值 IK 求解器 (P1.4) —— 策略层自研, 数值层用库 (Eigen SVD + KDL 雅可比)。
 *
 * 算法 = 种子阶梯 + DLS 主循环 + 零空间二级目标 (决策卡 §5.1):
 *   种子阶梯: 调用方种子 → 限位感知启发式 × N 随机重启 (TRAC-IK 配方) → 逐级启用
 *   DLS:      q̇ = J⁺(λ)·e, 阻尼伪逆经 SVD (Eigen), λ 随最小奇异值自适应
 *   零空间:   q̇ₙ = (I − J⁺J)·k∇H, H = w₁·限位中心距 + w₂·可操作度
 *             (限位是目标函数的一部分 —— xarm7 j4 家位距下限仅 11° 的教训)
 *   分支粘性: 全程不主动跳分支; 重启解与初始种子的关节距离超阈值 → 视为
 *             跳变, 报 ITERATION_LIMIT 而非输出 (跳变 = 关节瞬间大位移)
 *
 * 失败语义 (决策卡 §5.2): 失败时不修改 out_q —— 调用方保持上一解, 绝无
 * 部分解/猜测解混出。
 *
 * 线程: 求解全程本类无线程无锁, 但经 fk_ 间接引用 KDL 有状态求解器 ——
 * 跨线程并发调用需每线程独立持有 DlsIk+UrdfFk 实例 (详见 urdf_fk.hpp 用法契约)。
 * CM 双线程分工 (流式在 update / 冷启动在 worker) 据此各持一套。
 *
 * ARM_ANGLE 冗余偏好: 对偏置构型 (xarm7 实测非 S-R-S) 只是近似语义,
 * 第一版诚实拒绝 (UNSUPPORTED)。
 */
struct DlsIkConfig
{
	double pos_tolerance{1e-6};      // 位置收敛阈 [m]
	double rot_tolerance{1e-4};      // 姿态收敛阈 [rad] (1e-5 对流式无增益)
	int max_iterations{200};         // 单种子迭代硬上限 (级0与重启的默认值)
	int restart_max_iterations{0};   // 重启种子的迭代上限; 0=跟随 max_iterations。
	                                 // 预算受限冷启动的细调位 (2026-09-17 两轮实测, 420 样本):
	                                 //   w_manip=0 跳过 H₂ 数值微分后单迭代成本 ~1/3, 两档最优
	                                 //   统一为全局 max_iterations=40: 2ms 87.6%→95.5%,
	                                 //   5ms 94.5%→97.9% (级0烧穿预算会饿死整个阶梯);
	                                 //   分层(级0=200, 重启=40) 仅 91.4%@2ms —— 便宜迭代下
	                                 //   级0也该砍
	double lambda_base{0.01};        // DLS 阻尼基值 [m] (自适应在此之上)
	double lambda_max{0.5};          // 阻尼上限 (深奇异保护)
	double nullspace_gain{0.3};      // 零空间目标步长系数
	double w_center{1.0};            // H: 限位中心距权重
	double w_manip{0.0};             // H: 可操作度权重 (默认关: 数值微分梯度扰动收敛, 2026-09-17
	                                 //  实测; 且开启后每迭代 +2n 次雅可比 = 主成本。=0 时整块跳过零成本;
	                                 //  启用需解析梯度)
	int restart_count{40};           // 种子阶梯: 随机重启次数 (前半中心带分层采样, 后半全区间)
	double jump_threshold{1.5};       // [STREAMING] 解距种子最大 L1 距离 [rad] (粘性; 冷启动不检查)
	double w_rot{1.0};               // 姿态误差权重 (e=[e_p; w·e_r]: 米与弧度量纲平衡,
	                                 //  1.0 时姿态主导步长导致位置收敛慢/震荡)
	int max_backtrack{4};            // 线搜索回退次数 (步长减半; 治 DLS 走过解/震荡)
	uint64_t timeout_ns{0};          // 整次求解 (级0种子 + 全部重启) 的墙钟预算 [ns]; 0=不限时。
	                                 // 到时返回 ITERATION_LIMIT (失败不改输出), stats.timed_out=true。
	                                 // 检查粒度 = 单次迭代 (~40µs @xarm7), 实际超界 ≤ 1 迭代。
	                                 // RT 周期内调用时设为周期份额: 500Hz(2ms)→0.5~1ms, 200Hz(5ms)→1~2ms
	double near_singular_sigma{0.05}; // 失败分类阈值: 最佳失败尝试的 σ_min 低于此值 → NEAR_SINGULAR
	                                 // (2026-09-17 实测校准: 失败组 σ 全落 <0.02, 成功组 p25=0.067,
	                                 //  (0.02,0.067) 为天然空档, 取 0.05。CM 分流 = 结果码 × timed_out
	                                 //  组合: NEAR_SINGULAR+超时=奇异区被掐预算(worker 可救),
	                                 //  NEAR_SINGULAR+非超时=真不可解; σ 永不 gate 成功。
	                                 //  0=关闭分类; 分布账见 playbook §7)
};

/*
 * 求解模式 —— 粘性语义的场景化解耦 (2026-09-17 粘性误杀 31/90 冷启动样本后定稿):
 *   STREAMING  流式跟踪 (种子=上一解): 启用分支粘性 (解距种子 > jump_threshold
 *              视为跳分支拒绝输出, 拖动/伺服场景的连续性保证)。
 *   COLD_START 冷启动 (种子=分支代表/种子库): 禁用粘性 —— 解距种子远是常态
 *              不是错误 (实测失败样本的合法解距限位中心 9~11 rad)。
 * 调用方必须显式声明意图, 不从种子值隐式推断。
 */
enum class SolveMode : uint8_t
{
	STREAMING = 0,
	COLD_START = 1,
};

/*
 * 单次求解统计 (调用方可选消费; ulog/性能画像用)。
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
	                        // 成功=解处 (倒数第二次迭代的 σ, 末步在信任邻域内差异可忽略);
	                        // 失败=加权误差最低的失败尝试处; 粘性拒绝=被拒解处。
	                        // <0 = 本轮未进入迭代 (NOT_READY/UNREACHABLE 几何预检等)
};

class DlsIk
{
public:
	// 构造即持有 FK 库 (限位/雅可比来源); 不成功 ready_=false, solve 返回 NOT_READY
	[[nodiscard]] bool init(const UrdfFk * fk, std::string & message);

	/*
	 * 求解 (点 IK, 上游 ~100Hz 消费)。
	 * seed:   种子关节角 (jointCount 维; 建议传当前 q —— 流式连续性的根)
	 * red:    冗余偏好 (PRESERVE / LOCK_JOINT; ARM_ANGLE → UNSUPPORTED)
	 * out_q:  成功时写入解; 失败时不修改 (调用方保持上一解)
	 * stats:  可选出参 (求解统计; nullptr 忽略)
	 * 阻塞时长: 数十次迭代 × 每次一次 FK+雅可比, 典型 <1ms, 上限 max_iter×重启;
	 *           timeout_ns≠0 时被墙钟预算封顶 (超时 = ITERATION_LIMIT + stats.timed_out)
	 */
	[[nodiscard]] IkResult solve(
		const CartesianPose & target,
		const std::vector<double> & seed,
		const RedundancyPreference & red,
		std::vector<double> & out_q,
		DlsIkStats * stats = nullptr,
		SolveMode mode = SolveMode::STREAMING) const;

	[[nodiscard]] bool ready() const {return fk_ != nullptr;}

	// 配置注入 (参数与 FK 无关, 可随时更换, 影响下一次 solve)
	void setConfig(const DlsIkConfig & cfg) {cfg_ = cfg;}

	/*
	 * 种子库 (可选, 2026-09-17): COLD_START 阶梯的目标导向种子 —— 级0.5,
	 * 介于调用方种子与四分支之间; STREAMING 不使用 (warm 种子 + 粘性已是主路径)。
	 * 文件格式 (gen_seed_library.py 生成, 机型资产): 每行 "qw qx qy qz x y z | j1..jn br";
	 * 加载时逐条做限位 + FK 一致性校验 (关节序错配在此当场暴露)。
	 * 未加载 = 现行为 (分支代表 + 随机阶梯), 形态无关性不受影响。
	 */
	[[nodiscard]] bool loadSeedLibrary(const std::string & path, std::string & message);
	[[nodiscard]] bool seedLibraryLoaded() const {return !seed_lib_.empty();}
	// 已加载条目数 (0 = 未加载)
	[[nodiscard]] std::size_t seedLibrarySize() const {return seed_lib_.size();}

private:
	// 单次 DLS 求解 (给定种子, 无重启); deadline = 墙钟预算终点 (max()=不限时),
	// 每迭代首检查, 到时按迭代上限失败处理; iter_cap = 本种子的迭代上限;
	// sigma_out/err_out = 退出时的最新 σ_min / 加权误差 (可空; 调用方初始化为 -1)
	IkResult solveOnce(
		const CartesianPose & target, const std::vector<double> & seed,
		const RedundancyPreference & red, std::vector<double> & out_q,
		const std::chrono::steady_clock::time_point & deadline,
		int iter_cap,
		int * iterations_used = nullptr,
		double * sigma_out = nullptr,
		double * err_out = nullptr) const;

	// 零空间目标梯度 ∇H (限位中心距 + 可操作度, 数值微分可操作度项)
	void nullspaceGradient(
		const std::vector<double> & q, const RedundancyPreference & red,
		std::vector<double> & grad, UrdfFk::Scratch & fk_scratch) const;

	// 位姿误差: [位置误差 3; 姿态误差 3 (轴角向量)]
	static Eigen::Matrix<double, 6, 1> poseError(
		const CartesianPose & a, const CartesianPose & b);

	const UrdfFk * fk_{nullptr};
	DlsIkConfig cfg_;

	// 种子库数据 + 查询打分暂存 (实例每线程一份的契约下, solve() 内复用安全)
	struct SeedEntry
	{
		CartesianPose pose;
		std::vector<double> q;
		int branch{0};
	};
	std::vector<SeedEntry> seed_lib_;
	mutable std::vector<std::pair<double, std::size_t>> lib_scratch_;
};

}  // namespace unistackbot_controller
#endif  // UNISTACKBOT_CONTROLLER__DLS_IK_HPP_
