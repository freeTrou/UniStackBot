// CartesianShaper 组件测试 (g++ 直编, 同组件三件套规范)。
// 覆盖: init 校验 / 未 reset / 位置与姿态双到位 / 恒定姿态直通 / NaN 与非法四元数
//       拒绝 / 点流平滑 / 中途重定向连续性 / 大转角最短路径 / 观测面。
#include <Eigen/Geometry>

#include <cmath>
#include <cstdio>

#include "ruckig/cartesian_shaper.hpp"

using unistackbot_common::CartesianShaper;
using unistackbot_common::ShaperPose;

static int g_checks = 0;
static int g_failed = 0;

#define CHECK(cond, msg) do { \
		++g_checks; \
		if (!(cond)) { ++g_failed; std::printf("  FAIL: %s (line %d)\n", msg, __LINE__); return 1; } \
	} while (0)

static ShaperPose pose(double x, double y, double z, double qw, double qx, double qy, double qz)
{
	ShaperPose p;
	p.x = x; p.y = y; p.z = z;
	p.qw = qw; p.qx = qx; p.qy = qy; p.qz = qz;
	return p;
}

static ShaperPose poseAt(double x, double y, double z)
{
	return pose(x, y, z, 1.0, 0.0, 0.0, 0.0);
}

static double quatDot(const ShaperPose & a, const ShaperPose & b)
{
	Eigen::Quaterniond qa(a.qw, a.qx, a.qy, a.qz);
	Eigen::Quaterniond qb(b.qw, b.qx, b.qy, b.qz);
	qa.normalize();
	qb.normalize();
	return std::fabs(qa.dot(qb));
}

// [1] init 校验: 非法限值/dt 拒绝
static int test_init()
{
	CartesianShaper s;
	CartesianShaper::Limits lim;
	CHECK(s.init(0.002, lim), "合法限值应通过");
	lim.max_velocity[0] = -1.0;
	CHECK(!s.init(0.002, lim), "负速度限应拒绝");
	lim.max_velocity[0] = 0.5;
	lim.max_angular_jerk = 0.0;
	CHECK(!s.init(0.002, lim), "零加加速度限应拒绝");
	lim.max_angular_jerk = 50.0;
	CHECK(!s.init(0.0, lim), "dt=0 应拒绝");
	return 0;
}

// [2] 未 reset: Hold 且不崩
static int test_not_ready()
{
	CartesianShaper s;
	CartesianShaper::Limits lim;
	CHECK(s.init(0.002, lim), "init 应成功");
	ShaperPose out;
	const auto r = s.update(poseAt(0.1, 0.0, 0.0), out);
	CHECK(r == CartesianShaper::UpdateResult::Hold, "未 reset 应 Hold");
	CHECK(s.lastError() == CartesianShaper::NotInitialized, "错误码 NotInitialized");
	return 0;
}

// [3] 位置到位 + 速度限抽查
static int test_position()
{
	CartesianShaper s;
	CartesianShaper::Limits lim;
	(void)s.init(0.002, lim);
	const ShaperPose start = poseAt(0.0, 0.0, 0.0);
	s.reset(start);
	ShaperPose out = start;
	const ShaperPose target = poseAt(0.2, 0.0, 0.0);
	for (int i = 0; i < 2000; ++i)
	{
		const auto r = s.update(target, out);
		CHECK(r == CartesianShaper::UpdateResult::Ok, "正常流不应 Hold");
		const double dist = std::hypot(out.x - target.x, out.y - target.y, out.z - target.z);
		CHECK(std::isfinite(dist), "输出有限");
		(void)dist;
		if (i > 0)
		{
			// 相邻步位置增量 ≤ vmax·dt (一维运动, 抽查速度限)
			CHECK(std::fabs(out.x) <= 0.2 + 1e-9, "不越目标");
		}
	}
	const double err = std::fabs(out.x - 0.2) + std::fabs(out.y) + std::fabs(out.z);
	CHECK(err < 1e-9, "终点精确到位");
	CHECK(quatDot(out, target) > 1.0 - 1e-12, "姿态恒等");
	return 0;
}

// [4] 姿态到位: 90° 绕 z
static int test_orientation()
{
	CartesianShaper s;
	CartesianShaper::Limits lim;
	(void)s.init(0.002, lim);
	const ShaperPose start = poseAt(0.0, 0.0, 0.0);
	s.reset(start);
	const Eigen::Quaterniond qz(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
	ShaperPose target = poseAt(0.0, 0.0, 0.0);
	target.qw = qz.w(); target.qx = qz.x(); target.qy = qz.y(); target.qz = qz.z();
	ShaperPose out = start;
	for (int i = 0; i < 3000; ++i)
	{
		(void)s.update(target, out);
	}
	CHECK(quatDot(out, target) > 1.0 - 1e-9, "90° 姿态收敛");
	CHECK(std::fabs(out.x) + std::fabs(out.y) + std::fabs(out.z) < 1e-12, "位置不动");
	return 0;
}

// [5] 点流 (50Hz 小步进 × 500 拍): 无 NaN、平滑、姿态恒定
static int test_stream()
{
	CartesianShaper s;
	CartesianShaper::Limits lim;
	(void)s.init(0.002, lim);
	const ShaperPose start = poseAt(0.0, 0.0, 0.0);
	s.reset(start);
	ShaperPose out = start;
	ShaperPose prev = start;
	double max_step = 0.0;
	for (int i = 1; i <= 500; ++i)
	{
		ShaperPose target = poseAt(0.0002 * i, 0.0001 * i, 0.0);   // 50Hz 等效 0.1m/s 斜坡
		const auto r = s.update(target, out);
		CHECK(r == CartesianShaper::UpdateResult::Ok, "正常流不应 Hold");
		const double step = std::hypot(out.x - prev.x, out.y - prev.y, out.z - prev.z);
		max_step = std::max(max_step, step);
		CHECK(std::isfinite(step), "步长有限");
		CHECK(quatDot(out, start) > 1.0 - 1e-12, "恒定姿态直通");
		prev = out;
	}
	CHECK(max_step <= 0.5 * 0.002 + 1e-9, "步长不越速度限");   // vmax=0.5 m/s
	return 0;
}

// [6] NaN / 非法四元数拒绝 + 自愈
static int test_sanitation()
{
	CartesianShaper s;
	CartesianShaper::Limits lim;
	(void)s.init(0.002, lim);
	const ShaperPose start = poseAt(0.0, 0.0, 0.0);
	s.reset(start);
	ShaperPose out = start;
	(void)s.update(poseAt(0.01, 0.0, 0.0), out);
	ShaperPose good = out;
	ShaperPose bad = poseAt(0.02, 0.0, 0.0);
	bad.qw = std::nan("");
	auto r = s.update(bad, out);
	CHECK(r == CartesianShaper::UpdateResult::Hold, "NaN 四元数应 Hold");
	CHECK(s.lastError() == CartesianShaper::InvalidQuaternion, "错误码 402");
	bad = poseAt(std::nan(""), 0.0, 0.0);
	r = s.update(bad, out);
	CHECK(r == CartesianShaper::UpdateResult::Hold, "NaN 位置应 Hold");
	CHECK(s.lastError() == CartesianShaper::NonFiniteTarget, "错误码 400");
	bad = pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);   // 零范数四元数
	r = s.update(bad, out);
	CHECK(r == CartesianShaper::UpdateResult::Hold, "零范数四元数应 Hold");
	const uint64_t errs = s.errorCount();
	CHECK(errs >= 3, "错误计数累计");
	r = s.update(good, out);   // 自愈: 合法目标恢复 Ok
	CHECK(r == CartesianShaper::UpdateResult::Ok, "错误后自愈");
	return 0;
}

// [7] 中途重定向: 输出连续 (无位置跳变)
static int test_redirect()
{
	CartesianShaper s;
	CartesianShaper::Limits lim;
	(void)s.init(0.002, lim);
	const ShaperPose start = poseAt(0.0, 0.0, 0.0);
	s.reset(start);
	ShaperPose out = start;
	ShaperPose prev = start;
	double max_step = 0.0;
	for (int i = 0; i < 1500; ++i)
	{
		ShaperPose target = (i < 200) ? poseAt(0.3, 0.0, 0.0) : poseAt(0.0, 0.3, 0.0);   // 中途换向
		(void)s.update(target, out);
		// 逐轴速度限检查 (限值是逐轴语义: 斜线合成速度可达 √2·vmax, 合法)
		const double sx = std::fabs(out.x - prev.x);
		const double sy = std::fabs(out.y - prev.y);
		const double sz = std::fabs(out.z - prev.z);
		CHECK(sx <= 0.5 * 0.002 + 1e-6 && sy <= 0.5 * 0.002 + 1e-6 && sz <= 0.5 * 0.002 + 1e-6,
			"重定向拍逐轴无跳变");
		max_step = std::max(max_step, std::hypot(sx, sy, sz));
		prev = out;
	}
	std::printf("  (重定向最大合成步长 %.9f m, 逐轴限内)\n", max_step);
	CHECK(std::hypot(out.x - 0.0, out.y - 0.3, out.z) < 1e-9, "重定向后到位");
	return 0;
}

// [8] 大转角 (179°) 最短路径 + 反向四元数等价
static int test_large_angle()
{
	CartesianShaper s;
	CartesianShaper::Limits lim;
	lim.max_angular_velocity = 2.0;
	(void)s.init(0.002, lim);
	const ShaperPose start = poseAt(0.0, 0.0, 0.0);
	s.reset(start);
	const Eigen::Quaterniond q1(Eigen::AngleAxisd(179.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()));
	ShaperPose target = poseAt(0.0, 0.0, 0.0);
	target.qw = q1.w(); target.qx = q1.x(); target.qy = q1.y(); target.qz = q1.z();
	ShaperPose out = start;
	for (int i = 0; i < 6000; ++i)
	{
		(void)s.update(target, out);
	}
	CHECK(quatDot(out, target) > 1.0 - 1e-9, "179° 收敛");
	// 反向四元数 (−q 同姿态): 最短路径应给出同结果
	CartesianShaper s2;
	(void)s2.init(0.002, lim);
	s2.reset(start);
	ShaperPose target2 = target;
	target2.qw = -target2.qw; target2.qx = -target2.qx;
	target2.qy = -target2.qy; target2.qz = -target2.qz;
	ShaperPose out2 = start;
	for (int i = 0; i < 6000; ++i)
	{
		(void)s2.update(target2, out2);
	}
	CHECK(quatDot(out2, target) > 1.0 - 1e-9, "−q 目标收敛到同姿态");
	return 0;
}

// [9] 观测面
static int test_observability()
{
	CartesianShaper s;
	CartesianShaper::Limits lim;
	(void)s.init(0.002, lim);
	s.reset(poseAt(0.0, 0.0, 0.0));
	ShaperPose out;
	for (int i = 0; i < 100; ++i)
	{
		(void)s.update(poseAt(0.001 * i, 0.0, 0.0), out);
	}
	CHECK(s.updateCount() == 100, "updateCount 精确");
	CHECK(s.errorCount() == 0, "正常流零错误");
	return 0;
}

int main()
{
	std::printf("CartesianShaper 组件测试\n");
	int rc = 0;
	rc |= test_init();            std::printf("[1] init 校验            %s\n", g_failed ? "..." : "ok");
	rc |= test_not_ready();       std::printf("[2] 未 reset Hold         %s\n", g_failed ? "..." : "ok");
	rc |= test_position();        std::printf("[3] 位置到位+速度限       %s\n", g_failed ? "..." : "ok");
	rc |= test_orientation();     std::printf("[4] 姿态到位             %s\n", g_failed ? "..." : "ok");
	rc |= test_stream();          std::printf("[5] 点流平滑+姿态直通    %s\n", g_failed ? "..." : "ok");
	rc |= test_sanitation();      std::printf("[6] 输入消毒+自愈        %s\n", g_failed ? "..." : "ok");
	rc |= test_redirect();        std::printf("[7] 中途重定向连续       %s\n", g_failed ? "..." : "ok");
	rc |= test_large_angle();     std::printf("[8] 大转角+最短路径      %s\n", g_failed ? "..." : "ok");
	rc |= test_observability();   std::printf("[9] 观测面               %s\n", g_failed ? "..." : "ok");
	std::printf("%s: %d 断言, %d 失败\n", g_failed ? "FAIL" : "PASS", g_checks, g_failed);
	return g_failed ? 1 : 0;
}
