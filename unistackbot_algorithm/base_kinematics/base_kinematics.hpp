#ifndef UNISTACKBOT_ALGORITHM__BASE_KINEMATICS__BASE_KINEMATICS_HPP_
#define UNISTACKBOT_ALGORITHM__BASE_KINEMATICS__BASE_KINEMATICS_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace unistackbot_algorithm
{

/*
 * BaseKinematics —— 底盘运动学可插拔层 (2026-10-15, 底盘域 A2)。
 *
 * 定位: body twist ↔ 底盘执行通道 的闭式双向变换 (urdf_fk 同族: 纯函数、无迭代、
 * 无预算无收敛码 —— 与 IkSolver 那族迭代求解器不同族)。消费者 = BaseTwistController
 * (逆解: 整形后的 twist → 各通道命令; 正解: 通道反馈 → twist → odom 积分)。
 *
 * 硬契约 (IkSolver 五条同款风格):
 *   1. toActuators/fromActuators 输出向量由调用方预尺寸化 (actuatorCount()),
 *      热路径零分配零锁零 IO —— RT 环直调;
 *   2. init 失败不改实现状态并给出人话 err (fail-fast, 配置期暴露);
 *   3. 实现可有内部状态 (如舵轮过零翻转的最近角记忆) → 实例每控制器线程一份,
 *      toActuators 非 const;
 *   4. 非全向形态对不支持的 twist 分量的处理由实现声明 (如 diff 忽略 vy ——
 *      主流 diff_drive 同语义), 不报错不静默放大;
 *   5. 声明 (specs) 在 init 后不变; 通道顺序 = cfg.joints 顺序。
 *
 * 扩展 = 新实现子类 + 工厂表加一行 (A 主轴: 仓内注册制, 外部化挂账)。
 */

// base 系车体速度三分量: vx/vy [m/s], wz [rad/s] (REP-103 车体系约定)
struct BodyTwist
{
	double vx{0.0};
	double vy{0.0};
	double wz{0.0};
};

// 执行通道类型 (ros2_control 命令接口语义: 转向=position / 驱动=velocity)
enum class BaseActuatorType : uint8_t
{
	Position,
	Velocity,
};

// 通道声明 (init 后可查; specs 顺序 = 命令/反馈向量顺序 = cfg.joints 顺序)
struct BaseActuatorSpec
{
	std::string joint;
	BaseActuatorType type{BaseActuatorType::Velocity};
};

// 机型配置 (yaml → 控制器 → 此处): joints = 执行通道名全表;
// params = 几何参数, 实现自定义解读 (diff: [轮径 m, 轮距 m];
// n_steer: [轮径, 安装位 x×N, y×N, 转向零位偏置×N] —— 见其实现注释)
struct BaseKinConfig
{
	std::vector<std::string> joints;
	std::vector<double> params;
};

class BaseKinematics
{
public:
	virtual ~BaseKinematics() = default;

	// 配置期装配: 校验 joints/params 形状与几何合法性; 失败给出人话原因
	[[nodiscard]] virtual bool init(const BaseKinConfig & cfg, std::string & err) = 0;

	// 通道声明 (init 成功后有效)
	[[nodiscard]] virtual std::vector<BaseActuatorSpec> specs() const = 0;
	[[nodiscard]] virtual std::size_t actuatorCount() const = 0;

	// 全向能力声明: false = 非全向 (差速/阿克曼族) —— 控制器据此把 vy 指令
	// 钳零 (诚实拒绝横移, 不静默放大); 形态知识归插件, 控制器不猜
	[[nodiscard]] virtual bool holonomic() const = 0;

	// 逆解: twist → 各通道命令 (out 预尺寸化 actuatorCount(); 单位随通道类型:
	// position=rad, velocity=rad/s)
	virtual void toActuators(const BodyTwist & t, std::vector<double> & out) = 0;

	// 正解: 各通道反馈 (specs 顺序; position=rad 实测, velocity=rad/s 实测) →
	// 车体 twist (odom 积分消费)
	virtual void fromActuators(const std::vector<double> & fb, BodyTwist & out) const = 0;

	[[nodiscard]] virtual const char * name() const = 0;
};

// 注册 (组合根式, 懒初始化无静态序问题; 重复名 → false fail-fast)。
// 内置实现经各实现文件的 register<Name>() 自注册; 未来外部化 (挂账) 走同一扇门。
using BaseKinematicsMaker = std::unique_ptr<BaseKinematics> (*)();
[[nodiscard]] bool registerBaseKinematics(const std::string & name, BaseKinematicsMaker maker);

// 工厂 (未知名字 → nullptr + availableBaseKinematics 列可选项; 无默认纪律)
[[nodiscard]] std::unique_ptr<BaseKinematics> createBaseKinematics(const std::string & name);
[[nodiscard]] std::vector<std::string> availableBaseKinematics();

// 各实现文件的自注册入口 (ensureBuiltin 首次工厂调用时逐一执行)
void registerNSteerKinematics();

}  // namespace unistackbot_algorithm

#endif  // UNISTACKBOT_ALGORITHM__BASE_KINEMATICS__BASE_KINEMATICS_HPP_
