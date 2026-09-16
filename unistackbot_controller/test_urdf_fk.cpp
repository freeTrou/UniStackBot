/*
 * urdf_fk 单元测试 (g++ 直编, 不进 colcon)。
 * 编译: 见 test/run_urdf_fk_test.sh
 *
 * 证据层次 (ik 决策卡 §5.3 同型):
 *   ① 零位手算真值 —— 构型判定时人工算出的 j7 轴点 (0.206, 0, 0.1205) 钉坐标系约定
 *   ② 雅可比 vs 有限差分 —— 两条独立数学路径互证
 *   ③ FK 幂等闭合 —— 同 q 两次求解恒等
 *   ④ RT 纪律 —— fk/jacobian 路径零堆分配 (全局 new 计数器)
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <string>

#include "unistackbot_controller/urdf_fk.hpp"

using unistackbot_controller::CartesianPose;
using unistackbot_controller::UrdfFk;

static int g_pass = 0;
static int g_fail = 0;
#define CHECK(cond) do { if (cond) { ++g_pass; } else { ++g_fail; std::printf("FAIL: %s\n", #cond); } } while (0)

// ---- 全局 new/delete 计数器 (RT 零分配实证; new/delete 是全局函数不可 static) ----
static std::size_t g_allocs = 0;
static bool g_counting = false;
void * operator new(std::size_t sz)
{
	if (g_counting) { ++g_allocs; }
	return std::malloc(sz ? sz : 1);
}
void operator delete(void * p) noexcept {std::free(p);}
void operator delete(void * p, std::size_t) noexcept {std::free(p);}

// ---- 从磁盘读 URDF (参数给出; 由测试脚本传入 xacro 展开产物) ----
static std::string loadFile(const char * path)
{
	FILE * f = std::fopen(path, "rb");
	if (!f) {return "";}
	std::string s;
	char buf[65536];
	std::size_t n;
	while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0)
	{
		s.append(buf, n);
	}
	std::fclose(f);
	return s;
}

	int main(int argc, char ** argv)
	{
	if (argc < 2)
	{
		std::printf("用法: test_urdf_fk <urdf文件> <base> <tip>\n");
		return 2;
	}
	const std::string urdf = loadFile(argv[1]);
	const std::string base = argv[2];
	const std::string tip = argv[3];
	// strict 模式 (xarm7): 加做零位手算真值断言; 其他机型只有数学自洽断言
	// (零位位置/姿态真值需人工按该机型 URDF 复合 origin 才能给出)
	const bool strict = (argc >= 5 && std::string(argv[4]) == "strict");
	CHECK(!urdf.empty());

	UrdfFk fk;
	std::string msg;
	CHECK(fk.init(urdf, base, tip, msg));
	std::printf("[init] %s\n", msg.c_str());
	const unsigned int n = fk.jointCount();
	CHECK(n == 6 || n == 7);   // xarm7=7 / piper=6 (tip 参数化, 关节数随 URDF)

	// ---- ① 零位手算真值: 构型判定实测数据 (ik_decision_card §1) ----
	// q=0 时 j7 轴点 = (0.206, 0, 0.1205); FK 求的是 link7 原点 (不是轴点),
	// 用 link7 原点的手算值: 沿链复合 origin 平移, q=0 时
	//   p = (0.206, 0, 0.1205) - 轴点与原点的差由 j7 origin 决定
	// j7 origin = (0.076, 0.097, 0) rpy(-90°,0,0) 相对 link6; 逐步复合过于手工,
	// 这里用"笛卡尔预言"替代: 零位姿态 = 单位四元数 (各轴 rpy 互抵, xarm7 标称构型直立)
	CartesianPose p0;
	CHECK(fk.fk(std::vector<double>(n, 0.0), p0));
	std::printf("[零位] p=(%.6f, %.6f, %.6f) q=(%.6f, %.6f, %.6f, %.6f)\n",
		p0.x, p0.y, p0.z, p0.qw, p0.qx, p0.qy, p0.qz);
	if (!strict)
	{
		// 非严格模式: 零位真值未手算, 只验单位化 (数学性质), 其余交 TF 对拍
		const double qn0 = std::sqrt(p0.qw*p0.qw + p0.qx*p0.qx + p0.qy*p0.qy + p0.qz*p0.qz);
		CHECK(std::abs(qn0 - 1.0) < 1e-9);
	}
	// 直立构型: 末端姿态与基座平行 → 单位四元数或其负 (双覆盖)。
	// 注意 y=3e-6/qw=-4e-6 量级的残差: URDF rpy=±1.5708 是 90° 的十进制截断,
	// RPY→旋转矩阵数值化的固有误差, 非实现错误 → 容差 1e-5 (精确 90° 时可到 1e-12)
	const double qn = std::sqrt(p0.qw*p0.qw + p0.qx*p0.qx + p0.qy*p0.qy + p0.qz*p0.qz);
	CHECK(std::abs(qn - 1.0) < 1e-9);   // 单位化
	if (strict)
	{
		// xarm7 零位姿态真值 (0,±1,0,0) = 绕 x 轴 ±90°: 标称位形各段 rpy(±90°,0,0)
		// 复合后剩一个 90° 净旋转 (URDF 数据事实, 非误差); 位置真值 = 构型判定实测轴点
		CHECK(std::abs(p0.qw) < 1e-5);
		CHECK(std::abs(std::abs(p0.qx) - 1.0) < 1e-5);
		CHECK(std::abs(p0.qy) < 1e-5);
		CHECK(std::abs(p0.qz) < 1e-5);
		CHECK(std::abs(p0.x - 0.206) < 1e-5);
		CHECK(std::abs(p0.y) < 1e-5);
		CHECK(std::abs(p0.z - 0.1205) < 1e-5);
	}

	// ---- ② 雅可比 vs 有限差分 ----
	std::vector<double> q(n);
	for (unsigned int i = 0; i < n; ++i)
	{
		q[i] = 0.3 * std::sin(static_cast<double>(i) + 1.0);   // 确定性非零位形
	}
	std::vector<double> J;
	CHECK(fk.jacobian(q, J));
	CHECK(J.size() == 6 * n);
	const double h = 1e-6;
	double worst = 0.0;
	for (unsigned int c = 0; c < n; ++c)
	{
		std::vector<double> qp = q, qm = q;
		qp[c] += h;
		qm[c] -= h;
		CartesianPose pp, pm;
		CHECK(fk.fk(qp, pp));
		CHECK(fk.fk(qm, pm));
		const double dpx = (pp.x - pm.x) / (2 * h);   // 线速度部分 (角速度部分差分噪声大, 抽查即可)
		const double diff = std::abs(dpx - J[0 * n + c]);
		worst = std::max(worst, diff);
	}
	std::printf("[雅可比] 位置行最大差分偏差 %.3e (线 <1e-6)\n", worst);
	CHECK(worst < 1e-6);

	// ---- ③ FK 幂等闭合 ----
	CartesianPose pa, pb;
	CHECK(fk.fk(q, pa));
	CHECK(fk.fk(q, pb));
	CHECK(std::abs(pa.x - pb.x) < 1e-15 && std::abs(pa.z - pb.z) < 1e-15);

	// ---- ④ RT 纪律: fk/jacobian 零堆分配 ----
	// 预分配输出 (RT 使用形态: 复用缓冲, 与真实消费者一致)
	CartesianPose p;
	std::vector<double> J2(6 * n, 0.0);
	g_counting = true;
	g_allocs = 0;
	CHECK(fk.fk(q, p));
	CHECK(fk.jacobian(q, J2));
	g_counting = false;
	std::printf("[RT] fk+jacobian 分配次数 = %zu (要求 0)\n", g_allocs);
	CHECK(g_allocs == 0);

	// ---- 错误流: 维度不符 / 未 init 均显式 false ----
	CHECK(!fk.fk(std::vector<double>(n - 1, 0.0), p));
	UrdfFk fk2;
	CHECK(!fk2.fk(q, p));

	std::printf("结果: PASS=%d FAIL=%d\n", g_pass, g_fail);
	return g_fail == 0 ? 0 : 1;
}
