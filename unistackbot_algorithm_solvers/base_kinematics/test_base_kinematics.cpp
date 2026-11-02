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

	// ===== n_steer: 工厂清单 / N=3 (r1_pro 实际布局) 三场景 + 翻转 + 死区 =====
	{
		const auto avail = unistackbot_algorithm::availableBaseKinematics();
		bool has = false;
		for (const auto & n : avail) { has = has || n == "n_steer"; }
		CHECK(has, "available 含 n_steer");
	}
	{
		auto kin = createBaseKinematics("n_steer");
		CHECK(kin != nullptr, "工厂产出 n_steer");
		std::string e;
		BaseKinConfig bad;
		bad.joints = {"s1", "w1", "s2"};   // 奇数通道
		CHECK(!kin->init(bad, e), "奇数通道拒");
		// N=3: r1_pro 布局 (前二后一), r=0.0765, 零位偏置 0
		BaseKinConfig cfg;
		cfg.joints = {"steer1", "wheel1", "steer2", "wheel2", "steer3", "wheel3"};
		cfg.params = {0.0765,
			0.0, 0.0, 0.0,              // 零位偏置×3
			0.16897, 0.16897, -0.32703, // x
			0.28, -0.28, 0.0};          // y
		CHECK(kin->init(cfg, e), "N=3 合法配置");
		CHECK(kin->actuatorCount() == 6, "通道数 6");
		CHECK(kin->holonomic(), "舵轮族全向声明");
		const auto specs = kin->specs();
		CHECK(specs[0].type == unistackbot_algorithm::BaseActuatorType::Position && specs[1].type == unistackbot_algorithm::BaseActuatorType::Velocity, "steer=position/wheel=velocity");

		std::vector<double> cmd(6, 0.0);
		BodyTwist t2, back;

		// 纯直行: 各模块方向=0 (零位), 轮速 = vx/r
		t2 = {0.3, 0.0, 0.0};
		kin->toActuators(t2, cmd);
		CHECK(near(cmd[0], 0.0) && near(cmd[2], 0.0) && near(cmd[4], 0.0), "直行转角全零");
		CHECK(near(cmd[1], 0.3 / 0.0765) && near(cmd[3], 0.3 / 0.0765) && near(cmd[5], 0.3 / 0.0765), "直行等轮速");
		kin->fromActuators(cmd, back);
		CHECK(near(back.vx, 0.3, 1e-9) && near(back.vy, 0.0, 1e-9) && near(back.wz, 0.0, 1e-9), "直行正解回代");

		// 纯自旋: 模块方向 = 切向 (p×), 轮速 = ω|p|/r
		t2 = {0.0, 0.0, 1.0};
		kin->toActuators(t2, cmd);
		const double exp_a0 = std::atan2(0.16897, -0.28);   // v_i = wz·(-py, px) 方向
		const double d_dir = std::fabs(std::atan2(std::sin(cmd[0] - exp_a0), std::cos(cmd[0] - exp_a0)));
		const double d_flip = std::fabs(std::atan2(std::sin(cmd[0] - exp_a0 - M_PI), std::cos(cmd[0] - exp_a0 - M_PI)));
		CHECK(std::min(d_dir, d_flip) < 1e-9, "自旋模块0方向=切向 (含翻转)");
		kin->fromActuators(cmd, back);
		CHECK(near(back.vx, 0.0, 1e-9) && near(back.vy, 0.0, 1e-9) && near(back.wz, 1.0, 1e-9), "自旋正解回代");

		// 平移+自旋合成: 正解回代 (任意 twist 往返一致)
		t2 = {0.2, -0.1, 0.4};
		kin->toActuators(t2, cmd);
		kin->fromActuators(cmd, back);
		CHECK(near(back.vx, 0.2, 1e-9) && near(back.vy, -0.1, 1e-9) && near(back.wz, 0.4, 1e-9), "合成 twist 回代");

		// 过零翻转: 上次 -3.0 rad (近 -π), 新目标方向 +0.14 (≈ -3.0+π 更近) → 翻转取近角
		{
			auto k2 = createBaseKinematics("n_steer");
			BaseKinConfig c2;
			c2.joints = {"s1", "w1", "s2", "w2"};
			c2.params = {0.1, 0.0, 0.0, 0.2, 0.3, 0.0, -0.3};   // N=2, 模块 (0.2,0.3)/(-0.2,-0.3)
			CHECK(k2->init(c2, e), "N=2 合法");
			std::vector<double> c(4, 0.0);
			// 第一腿: vx=-0.3 → 方向角 π; 距初始 0 为 π, 翻转 (α+π→0) 距离 0 → 翻转!
			// 结果 steer=0, 轮速反号 (-3): 车倒着走达成"向 -x" —— 少转半圈
			BodyTwist tt{-0.3, 0.0, 0.0};
			k2->toActuators(tt, c);
			CHECK(near(c[0], 0.0, 1e-9), "翻转: 方向 π 从 0 出发改走 0 (α+π 等效取近)");
			CHECK(c[1] < 0.0, "翻转: 轮速反号");
			// 第二腿: vx=+0.3 → 方向 0, 与上次 0 距离 0 → 不翻, 轮速正
			BodyTwist tt2{0.3, 0.0, 0.0};
			k2->toActuators(tt2, c);
			CHECK(near(c[0], 0.0, 1e-9) && c[1] > 0.0, "同向续行不翻");
			// 直角目标: vy=+0.3 → 方向 π/2, 距 0 与翻转各 π/2 (边界不翻) → 直接转
			BodyTwist tt3{0.0, 0.3, 0.0};
			k2->toActuators(tt3, c);
			CHECK(near(c[0], M_PI / 2.0, 1e-9), "直角目标直接转 (无翻转利益)");
		}

		// 低速死区: twist≈0 → 转角保持 + 轮速零
		{
			std::vector<double> c(4, 0.0);
			BodyTwist tz{0.0, 0.0, 0.0};
			auto k3 = createBaseKinematics("n_steer");
			BaseKinConfig c3;
			c3.joints = {"s1", "w1", "s2", "w2"};
			c3.params = {0.1, 0.0, 0.0, 0.2, 0.3, 0.0, -0.3};
			CHECK(k3->init(c3, e), "死区实例 init");
			k3->toActuators(tz, c);
			CHECK(near(c[0], 0.0, 1e-12) && near(c[2], 0.0, 1e-12), "死区: 转角保持初始 (未动过)");
			CHECK(near(c[1], 0.0) && near(c[3], 0.0), "死区: 轮速零");
		}
	}

	std::printf("BaseKinematics: PASS=%d FAIL=%d\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
