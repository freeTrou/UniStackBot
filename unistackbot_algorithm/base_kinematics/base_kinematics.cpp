#include "base_kinematics/base_kinematics.hpp"

#include <cmath>
#include <map>

namespace unistackbot_algorithm
{
namespace
{

/*
 * DiffKinematics —— 两轮差速 (2026-10-15, 底盘域 A3 参考实现一)。
 *
 * cfg.joints = [左轮, 右轮] (均 velocity 通道);
 * cfg.params = [轮半径 r (m), 轮距 L (m)] —— 轮距 = 两驱动轮中心距。
 *
 * 逆解 (轮角速度 rad/s):  v_l = (vx - wz·L/2)/r,  v_r = (vx + wz·L/2)/r
 * 正解 (odom):            vx = (v_l+v_r)/2·r,  wz = (v_r-v_l)·r/L,  vy≡0
 *
 * 非全向声明 (契约 4): vy 分量忽略 —— 差速物理上无横移, 主流 diff_drive 同语义;
 * 调用方 (控制器) 的限幅层负责把整机 max_lin 收到本形态可达域, 不在此静默放大。
 */
class DiffKinematics final : public BaseKinematics
{
public:
	[[nodiscard]] bool init(const BaseKinConfig & cfg, std::string & err) override
	{
		if (cfg.joints.size() != 2)
		{
			err = "diff 需要 joints=[左轮, 右轮] 共 2 通道, 实得 " + std::to_string(cfg.joints.size());
			return false;
		}
		if (cfg.params.size() != 2 || !std::isfinite(cfg.params[0]) || !(cfg.params[0] > 0.0) || !std::isfinite(cfg.params[1]) || !(cfg.params[1] > 0.0))
		{
			err = "diff 需要 params=[轮半径(m), 轮距(m)] 且均为正有限值";
			return false;
		}
		r_ = cfg.params[0];
		track_ = cfg.params[1];
		specs_ = {{cfg.joints[0], BaseActuatorType::Velocity}, {cfg.joints[1], BaseActuatorType::Velocity}};
		return true;
	}

	[[nodiscard]] std::vector<BaseActuatorSpec> specs() const override {return specs_;}
	[[nodiscard]] std::size_t actuatorCount() const override {return 2;}
	[[nodiscard]] bool holonomic() const override {return false;}   // 差速无横移

	void toActuators(const BodyTwist & t, std::vector<double> & out) override
	{
		out[0] = (t.vx - t.wz * track_ * 0.5) / r_;
		out[1] = (t.vx + t.wz * track_ * 0.5) / r_;
	}

	void fromActuators(const std::vector<double> & fb, BodyTwist & out) const override
	{
		out.vx = (fb[0] + fb[1]) * 0.5 * r_;
		out.vy = 0.0;   // 非全向: 横移物理不存在
		out.wz = (fb[1] - fb[0]) * r_ / track_;
	}

	[[nodiscard]] const char * name() const override {return "diff";}

private:
	double r_{0.1};
	double track_{0.4};
	std::vector<BaseActuatorSpec> specs_;
};

// 注册表 (A 主轴组合根式: 懒初始化 + 公共注册函数; 加内置实现 = ensureBuiltin 加一行;
// 外部化挂账走同一扇门 registerBaseKinematics)
std::map<std::string, BaseKinematicsMaker> & registry()
{
	static std::map<std::string, BaseKinematicsMaker> reg;
	return reg;
}

}  // namespace

bool registerBaseKinematics(const std::string & name, BaseKinematicsMaker maker)
{
	return registry().emplace(name, maker).second;   // 重复名 false (组合根 bug fail-fast)
}

// 内置实现自注册 (首次工厂调用执行一次; 函数显式调用, 不用静态初始化器 —— dead-strip 防线)
static void ensureBuiltinKinematics()
{
	static bool done = false;
	if (done)
	{
		return;
	}
	done = true;
	(void)registerBaseKinematics("diff", []() -> std::unique_ptr<BaseKinematics> {return std::make_unique<DiffKinematics>(); });
	registerNSteerKinematics();
}

std::unique_ptr<BaseKinematics> createBaseKinematics(const std::string & name)
{
	ensureBuiltinKinematics();
	const auto it = registry().find(name);
	return it == registry().end() ? nullptr : it->second();
}

std::vector<std::string> availableBaseKinematics()
{
	ensureBuiltinKinematics();
	std::vector<std::string> names;
	for (const auto & kv : registry())
	{
		names.push_back(kv.first);
	}
	return names;
}

}  // namespace unistackbot_algorithm
