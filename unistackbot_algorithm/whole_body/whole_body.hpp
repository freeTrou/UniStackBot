#ifndef UNISTACKBOT_ALGORITHM__WHOLE_BODY_HPP_
#define UNISTACKBOT_ALGORITHM__WHOLE_BODY_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "urdf_fk/urdf_fk.hpp"   // CartesianPose

namespace unistackbot_algorithm
{

/*
 * 全身/上身多任务解算缝契约 (2026-10-29 三律通过; v0.1 草案 —— 待与 mos 维护者 co-sign 定稿,
 * 定稿前本签名不消费不落地; mos/docs ODE §3.2/附录 B-C/E 对账记录)。
 * 修订: v0.3 (2026-10-30 用户批) —— urdf_path → urdf (XML 文本; ros2_control 全生态
 * 字符串形态背书: robot_description 参数/component_parser/话题皆字符串, 路径是异类)
 * + 三条约定成文 (control_joints 声明序 = 一切数组序 / 空 vendor 语义 / 限位数据源纪律);
 * v0.2 (2026-10-29 用户批) —— WholeBodyConfig 加 base_frame (目标参考系显性化:
 * 原注释"base 系"是隐性假设, 实现须校验存在不许忽略) + dt_s (worker 节拍约定值;
 * 单一真相在壳——装配期壳对照自身节拍校验, 不一致拒启);
 * v0.1 (2026-10-29 用户批) —— solve 返回 bool → WholeBodyResult 枚举
 * (参考实现示例暴露的缺口①收口); TaskState 值名对齐仓内枚举 CAPS 先例 (IkResult/SolveMode)。
 *
 * 来源 = mos arm_controller WbcSolver 实物形状提取: 双臂耦合解算 (HQP) 数学完整保留;
 * 它的线程/组装胶水 (异步 set/get + 内部 mutex + 头通道) 由壳的等价机制替代——契约只收数学。
 *
 * 解算域: **worker** —— 壳起线程按自有节拍调 solve(), 输入输出经无锁最新值通道交接
 * (机制归壳), 求解器不知道任何通道/线程/锁的存在 (硬规则③ 解算域是壳的属性;
 * 消息驱动是 CM 的形状, 不适合全身连续伺服 —— 两者是并列解算域, 非上下级)。
 *
 * 拓扑裁决定案 (律 2): 头不进本契约 —— 实码证实头在 mos 本就是独立通道 (head_otg_
 * 速度整形 + has_head_joints_ 可选, 不进 QP), 归宿 = JS 域 js_head 编组 (零新接口);
 * 双臂拓扑同样不进签名 —— goals 是集合, 单臂=1 双臂=2, 同一实现零改动装载
 * (契约三铁律: 拓扑假设不进契约)。
 *
 * 求解器实现契约 (安全相关, 违反 = 控制链事故; IkSolver 五条同款精神):
 *
 * 1. **失败绝不改 out** —— 无部分解混出; 调用方保持上一解是安全语义的根。
 * 2. **同步纯函数姿态** —— 零线程/零锁/零 ROS; 实例线程私有 (数学暂存跨线程互踩,
 *    IkSolver 契约 4 同款); RT 环禁入 (worker 域专用, 实现须自声明)。
 * 3. **init 指纹选拒不代选** —— control_joints/end_frames 与 URDF/厂商配置不符 =
 *    false + why 拒启 (无默认纪律; mos 的 nv/2 对半假设与 14 轴名 ABI 在此手术为显式配置)。
 * 4. **诚实拒绝进 TaskDiag** —— kUnreachable/kLimitConflict 是验收面不是异常
 *    (吸收 mos isCartesianTargetReachable; IkResult 语义同款)。
 * 5. **gravity 结果内带** —— mos ControlResult 形状原样保留 (前馈通道)。
 *
 * 实现接入 = ik_plugins 同款机制 (PLUGINLIB_EXPORT_CLASS + plugin.xml + cmake export,
 * 命名按能力家族如 upper_body_hqp, 不按机型)。对外插件缝纪律: 本接口是跨仓库二进制
 * 边界 —— 虚函数签名变更是破坏性变更; 新增只允许带默认实现追加在末尾, 不插序/不改/
 * 不删; solve 不得抛异常 (vendored 数学芯的异常在适配层翻译为失败返回)。
 */

/* 任务结局 —— 连续伺服语境: 逼近中是常态不是错误; 后两值 = 诚实拒绝 (契约 4)。 */
enum class TaskState : uint8_t
{
	CONVERGED = 0,      // 误差进容差
	APPROACHING = 1,    // 正常逼近中 (连续伺服的稳态)
	UNREACHABLE = 2,    // 目标不可达
	LIMIT_CONFLICT = 3, // 目标与限位/约束冲突
};

struct CartesianGoal
{
	std::string frame;         // ∈ WholeBodyConfig::end_frames
	CartesianPose target;      // base 系 = WholeBodyConfig.base_frame (与 CM ~/command 同系)
};

struct FramePose
{
	std::string frame;
	CartesianPose pose;
};

struct TaskDiag
{
	std::string task;                                   // 任务名 (end_frame 名或实现私有任务名)
	double error{0.0};                                  // 当前误差范数 (领域量)
	TaskState state{TaskState::APPROACHING};
};

/*
 * 整次解算的分层结果码 —— IkResult 同款先例 (错误跟着产生它的层走; v0.1 bool→枚举,
 * 用户批 2026-10-29)。失败一律不改 out (契约 1 不变), 枚举只补"为什么"通道:
 * 壳据此分流——NOT_READY=装配错误 (致命); UNREACHABLE/LIMIT_CONFLICT=目标侧问题
 * (可反馈给上层用户); SOLVE_FAILED=保持上一解+计数观察。长文本给日志, 短码给程序逻辑。
 */
enum class WholeBodyResult : uint8_t
{
	OK = 0,               // out 完整写入
	NOT_READY = 1,        // 未 init / 维度门失败 (装配错误, 致命)
	UNREACHABLE = 2,      // 目标不可达 (末端出安全可达域 / 目标非有限)
	LIMIT_CONFLICT = 3,   // 约束不可行 (限位/刹车包络冲突)
	SOLVE_FAILED = 4,     // QP 失败 / 层级违约 (保持上一解)
};

// 每码对应的标准信息 (单一事实源, 日志侧调用; ik_result_message 同款)
[[nodiscard]] constexpr const char * whole_body_result_message(WholeBodyResult r) noexcept
{
	switch (r)
	{
		case WholeBodyResult::OK:
		{
			return "解算成功";
		}
		case WholeBodyResult::NOT_READY:
		{
			return "求解器未就绪或维度不符";
		}
		case WholeBodyResult::UNREACHABLE:
		{
			return "目标不可达";
		}
		case WholeBodyResult::LIMIT_CONFLICT:
		{
			return "约束不可行 (限位/刹车包络冲突)";
		}
		case WholeBodyResult::SOLVE_FAILED:
		{
			return "QP 失败或层级违约";
		}
	}
	return "未知结果码";
}

/*
 * 装配配置 —— 全部显式无默认 (yaml `whole_body:` 块装载, 壳组装后传本结构)。
 * vendor_config_path 是实现的私有领地 (权重/优先级/容差), 契约不解释其内容。
 */
/*
 * 装配配置 —— 全部显式无默认。
 * 序语义 (v0.3 成文): control_joints 声明序 = 输入 q/dq 与输出 q/dq/ddq/gravity 的
 * 一切数组序; 关节名重复/不在 URDF = init 拒。
 * 限位数据源纪律 (v0.3 成文): 速度/加速度限住 vendor 参数, 勿信 URDF 的 velocity 属性
 * (导入 URDF 常为转换器默认值——arm7_test 实证七关节全同值); URDF 可信几何与位置限位。
 */
struct WholeBodyConfig
{
	std::string urdf;                          // URDF XML 文本 (v0.3; 原为 urdf_path——ros2_control 全生态 URDF 走字符串, 壳零转换直传)
	std::string base_frame;                    // 全部 CartesianGoal/FramePose 的参考系 (v0.2; 实现须校验存在于模型, 不许忽略)
	std::vector<std::string> control_joints;   // 显式索引集 (替 nv/2 对半假设; 双臂=14, 单臂=7)
	std::vector<std::string> end_frames;       // 末端集合 (替"左右手"双参签名)
	std::vector<std::string> telemetry_frames; // 遥测帧表 (替 getLeftHandPose 家族; 合法集合由实现声明, 未知名 init 拒)
	double dt_s;                               // worker 节拍约定值 (v0.2; 壳装配期对照自身节拍校验, 不一致拒启——单一真相在壳)
	std::string vendor_config_path;            // 实现私有 (权重/优先级/容差)。与 urdf 的不对称是设计: URDF 是运行时参数 (字符串), 厂商配置是部署资产 (路径)。空 = 本实现无厂商参数; 实现必需而空 = init 拒
};

struct WholeBodyInput
{
	std::vector<double> q;                     // 测量 (control_joints 序; 无速度测量时 dq 由壳传零)
	std::vector<double> dq;                    // 测量 (control_joints 序)
	std::vector<CartesianGoal> goals;          // 最新目标集合 (壳经值通道交接, 求解器不知情)
};

struct WholeBodyOutput
{
	std::vector<double> q, dq, ddq;            // 关节指令 (control_joints 序)
	std::vector<double> gravity;               // 结果内带重力矩 (control_joints 序; 契约 5)
	std::vector<FramePose> frames;             // telemetry_frames 当前位姿 (遥测随解出)
	std::vector<TaskDiag> tasks;               // 各任务诊断 (诚实拒绝面)
};

class WholeBodySolver
{
public:
	virtual ~WholeBodySolver() = default;

	// 指纹选拒 (契约 3): joints/frames 与 URDF/厂商配置不符 = false + why, 不代选不猜测
	[[nodiscard]] virtual bool init(const WholeBodyConfig & config, std::string & why) = 0;

	/*
	 * 同步解算 (壳 worker 线程按自有节拍调用; 连续伺服 = 每次喂最新 input)。
	 * OK = out 完整写入; 其余 = out 不修改, 调用方保持上一解 (契约 1),
	 * 码即失败原因 (壳分流见 WholeBodyResult 注释)。
	 * 温启动状态住实现内部 (实例线程私有, 契约 2)。
	 */
	[[nodiscard]] virtual WholeBodyResult solve(const WholeBodyInput & in, WholeBodyOutput & out) const = 0;
};

}  // namespace unistackbot_algorithm
#endif  // UNISTACKBOT_ALGORITHM__WHOLE_BODY_HPP_
