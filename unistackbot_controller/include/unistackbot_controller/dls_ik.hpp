#ifndef UNISTACKBOT_CONTROLLER__DLS_IK_HPP_
#define UNISTACKBOT_CONTROLLER__DLS_IK_HPP_

#include <memory>
#include <string>
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
 * ARM_ANGLE 冗余偏好: 对偏置构型 (xarm7 实测非 S-R-S) 只是近似语义,
 * 第一版诚实拒绝 (UNSUPPORTED)。
 */
struct DlsIkConfig
{
	double pos_tolerance{1e-6};      // 位置收敛阈 [m]
	double rot_tolerance{1e-4};      // 姿态收敛阈 [rad] (1e-5 对流式无增益)
	int max_iterations{200};         // 单次求解硬上限
	double lambda_base{0.01};        // DLS 阻尼基值 [m] (自适应在此之上)
	double lambda_max{0.5};          // 阻尼上限 (深奇异保护)
	double nullspace_gain{0.3};      // 零空间目标步长系数
	double w_center{1.0};            // H: 限位中心距权重
	double w_manip{0.0};             // H: 可操作度权重 (默认关: 数值微分梯度扰动收敛, 2026-09-17 实测; 启用需解析梯度)
	int restart_count{40};           // 种子阶梯: 随机重启次数 (配合停滞提前退出, 单种子最坏 ~1ms)
	double jump_threshold{1.5};      // 重启解与原种子的最大关节距离 [rad] (粘性)
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
	 * 阻塞时长: 数十次迭代 × 每次一次 FK+雅可比, 典型 <1ms, 上限 max_iter×重启
	 */
	[[nodiscard]] IkResult solve(
		const CartesianPose & target,
		const std::vector<double> & seed,
		const RedundancyPreference & red,
		std::vector<double> & out_q) const;

	[[nodiscard]] bool ready() const {return fk_ != nullptr;}

private:
	// 单次 DLS 求解 (给定种子, 无重启); 返回解/失败码
	IkResult solveOnce(
		const CartesianPose & target, const std::vector<double> & seed,
		const RedundancyPreference & red, std::vector<double> & out_q) const;

	// 零空间目标梯度 ∇H (限位中心距 + 可操作度, 数值微分可操作度项)
	void nullspaceGradient(
		const std::vector<double> & q, const RedundancyPreference & red,
		std::vector<double> & grad) const;

	// 位姿误差: [位置误差 3; 姿态误差 3 (轴角向量)]
	static Eigen::Matrix<double, 6, 1> poseError(
		const CartesianPose & a, const CartesianPose & b);

	const UrdfFk * fk_{nullptr};
	DlsIkConfig cfg_;
};

}  // namespace unistackbot_controller
#endif  // UNISTACKBOT_CONTROLLER__DLS_IK_HPP_
