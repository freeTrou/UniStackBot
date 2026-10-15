/* BaseKinematics 单元测试 (2026-10-15, 底盘域 A3)。
 * g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -I.. base_kinematics.cpp test_base_kinematics.cpp \
 *     -o /tmp/t_bk && /tmp/t_bk   (在 base_kinematics/ 目录下; 纯 std 零依赖)
 *
 * 覆盖: 工厂 (未知名/清单) / diff 配置校验 / 直行-原地转-圆弧三场景数值 /
 *       逆正解回代一致性 / 非全向声明 (vy=0) */
#include <cmath>
#include <cstdio>

#include "base_kinematics/base_kinematics.hpp"

using unistackbot_algorithm::BaseKinConfig;
using unistackbot_algorithm::BaseKinematics;
using unistackbot_algorithm::BodyTwist;
using unistackbot_algorithm::createBaseKinematics;

static int g_pass = 0, g_fail = 0;
#define CHECK(cond, msg) do { if (cond) { ++g_pass; } else { ++g_fail; std::printf("FAIL: %s\n", msg); } } while (0)
static bool near(double a, double b, double eps = 1e-12) {return std::fabs(a - b) <= eps;}

int main()
{
	// ---- 工厂: 未知名 nullptr + 清单含 diff ----
	{
		CHECK(createBaseKinematics("__none__") == nullptr, "未知名 nullptr");
		const auto avail = unistackbot_algorithm::availableBaseKinematics();
		bool has = false;
		for (const auto & n : avail) { has = has || n == "diff"; }
		CHECK(has, "available 含 diff");
	}

	auto kin = createBaseKinematics("diff");
	CHECK(kin != nullptr, "工厂产出 diff");

	std::string err;
	BaseKinConfig cfg;
	cfg.joints = {"left_wheel", "right_wheel"};
	cfg.params = {0.1, 0.4};   // r=0.1m, L=0.4m

	// ---- 配置校验: 通道数/几何非法拒 ----
	{
		auto bad = cfg; bad.joints = {"only_one"};
		CHECK(!kin->init(bad, err) && !err.empty(), "通道数错拒 + err 人话");
		auto bad2 = cfg; bad2.params = {0.1, -0.4};
		CHECK(!kin->init(bad2, err), "负轮距拒");
		auto bad3 = cfg; bad3.params = {0.0, 0.4};
		CHECK(!kin->init(bad3, err), "零轮径拒");
		CHECK(kin->init(cfg, err), "合法配置过");
		CHECK(kin->actuatorCount() == 2, "通道数 2");
		const auto specs = kin->specs();
		CHECK(specs[0].joint == "left_wheel" && specs[1].joint == "right_wheel", "通道名顺序");
		CHECK(specs[0].type == unistackbot_algorithm::BaseActuatorType::Velocity, "差速通道=velocity");
	}

	std::vector<double> cmd(2), fb(2);
	BodyTwist t, back;

	// ---- 直行: vx=0.5, wz=0 → 双轮等速 5 rad/s ----
	t = {0.5, 0.0, 0.0};
	kin->toActuators(t, cmd);
	CHECK(near(cmd[0], 5.0) && near(cmd[1], 5.0), "直行等轮速");
	kin->fromActuators(cmd, back);
	CHECK(near(back.vx, 0.5) && near(back.wz, 0.0), "直行回代");

	// ---- 原地转: vx=0, wz=1.0 → 双轮反号 ±2 rad/s ----
	t = {0.0, 0.0, 1.0};
	kin->toActuators(t, cmd);
	CHECK(near(cmd[0], -2.0) && near(cmd[1], 2.0), "原地转反号轮速");
	kin->fromActuators(cmd, back);
	CHECK(near(back.vx, 0.0) && near(back.wz, 1.0), "原地转回代");

	// ---- 圆弧: vx=0.3, wz=0.5 → 左 1.0 / 右 2.0 ----
	t = {0.3, 0.0, 0.5};
	kin->toActuators(t, cmd);
	CHECK(near(cmd[0], (0.3 - 0.1) / 0.1) && near(cmd[1], (0.3 + 0.1) / 0.1), "圆弧轮速");
	kin->fromActuators(cmd, back);
	CHECK(near(back.vx, 0.3) && near(back.wz, 0.5), "圆弧回代");

	// ---- 非全向声明: vy 输入被忽略, 正解 vy 恒 0 ----
	t = {0.2, 0.7, 0.1};
	kin->toActuators(t, cmd);
	kin->fromActuators(cmd, back);
	CHECK(near(back.vy, 0.0), "正解 vy≡0 (非全向诚实)");
	CHECK(near(back.vx, 0.2) && near(back.wz, 0.1), "vy 忽略不污染 vx/wz");

	std::printf("BaseKinematics: PASS=%d FAIL=%d\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
