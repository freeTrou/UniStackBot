/*
 * 球腕指纹判定测试 (g++ 直编, 不进 colcon)。
 *
 * 用例:
 *   [1] 合成正例: 内置 6R 链, 末端三轴交于一点 → spherical=true
 *   [2] 合成反例: 同链 joint6 加 5cm 垂直偏移 → spherical=false
 *   [3] piper 实测: xacro 展开的真实 URDF (脚本传入) → spherical=true,
 *       残差应为 88µm 量级 (决策卡 §1 的几何实锤回验)
 *
 * 用法: ./test_wrist_fingerprint <piper.urdf>
 */
#include <cstdio>
#include <cstdlib>
#include <string>

#include "analytic_piper/analytic_piper.hpp"
#include "analytic_piper/ikfast_include.h"
#include "urdf_fk/urdf_fk.hpp"

using unistackbot_algorithm::UrdfFk;
using unistackbot_algorithm::check_spherical_wrist;
using unistackbot_algorithm::WristFingerprint;
using unistackbot_interface::RedundancyPreference;

extern "C" void ComputeFk(const IkReal * j, IkReal * eetrans, IkReal * eerot);

namespace
{
constexpr double kTol = 0.001;   // 与实现内常量同档 (1mm)

// 内置合成 6R 链: 前三关节构造任意臂形, j6 沿 x5 (垂直于自身轴) 偏移与否
std::string synthetic_urdf(double j6_offset_x)
{
	std::string s = "<?xml version=\"1.0\"?><robot name=\"t\"><link name=\"base\"/>";
	for (int i = 1; i <= 6; ++i)
	{
		s += "<link name=\"l" + std::to_string(i) + "\"/>";
	}
	s += "<link name=\"tool\"/>";
	// j1 绕 z @base; j2 绕 y @z0.2; j3 绕 y @y0.3 (构造非退化臂形)
	// 注: revolute 必须带 <limit> (kdl_parser 硬要求)
	s += "<joint name=\"j1\" type=\"revolute\"><parent link=\"base\"/><child link=\"l1\"/>"
	     "<origin xyz=\"0 0 0\" rpy=\"0 0 0\"/><axis xyz=\"0 0 1\"/>"
	     "<limit lower=\"-3\" upper=\"3\" effort=\"0\" velocity=\"0\"/></joint>";
	s += "<joint name=\"j2\" type=\"revolute\"><parent link=\"l1\"/><child link=\"l2\"/>"
	     "<origin xyz=\"0 0 0.2\" rpy=\"0 0 0\"/><axis xyz=\"0 1 0\"/>"
	     "<limit lower=\"-3\" upper=\"3\" effort=\"0\" velocity=\"0\"/></joint>";
	s += "<joint name=\"j3\" type=\"revolute\"><parent link=\"l2\"/><child link=\"l3\"/>"
	     "<origin xyz=\"0 0.3 0\" rpy=\"0 0 0\"/><axis xyz=\"0 1 0\"/>"
	     "<limit lower=\"-3\" upper=\"3\" effort=\"0\" velocity=\"0\"/></joint>";
	// 末端三轴: j4 绕 z @x0.25; j5 绕 y @z0.1 (过 j4 原点); j6 垂直偏移 j6_offset_y
	s += "<joint name=\"j4\" type=\"revolute\"><parent link=\"l3\"/><child link=\"l4\"/>"
	     "<origin xyz=\"0.25 0 0.1\" rpy=\"0 0 0\"/><axis xyz=\"0 0 1\"/>"
	     "<limit lower=\"-3\" upper=\"3\" effort=\"0\" velocity=\"0\"/></joint>";
	s += "<joint name=\"j5\" type=\"revolute\"><parent link=\"l4\"/><child link=\"l5\"/>"
	     "<origin xyz=\"0 0 0\" rpy=\"1.5708 0 0\"/><axis xyz=\"0 0 1\"/>"
	     "<limit lower=\"-3\" upper=\"3\" effort=\"0\" velocity=\"0\"/></joint>";
	s += "<joint name=\"j6\" type=\"revolute\"><parent link=\"l5\"/><child link=\"l6\"/>"
	     "<origin xyz=\"" + std::to_string(j6_offset_x) + " 0 0\" rpy=\"-1.5708 0 0\"/>"
	     "<axis xyz=\"0 0 1\"/><limit lower=\"-3\" upper=\"3\" effort=\"0\" velocity=\"0\"/></joint>";
	s += "<joint name=\"fix\" type=\"fixed\"><parent link=\"l6\"/><child link=\"tool\"/>"
	     "<origin xyz=\"0 0 0.05\" rpy=\"0 0 0\"/></joint></robot>";
	return s;
}

int check_case(const char * tag, const std::string & urdf, bool expect_spherical,
	const char * base, const char * tip)
{
	UrdfFk fk;
	std::string msg;
	if (!fk.init(urdf, base, tip, msg))
	{
		std::printf("FAIL[%s]: UrdfFk init 失败: %s\n", tag, msg.c_str());
		return 1;
	}
	const WristFingerprint fp = check_spherical_wrist(fk, kTol);
	const bool pass = (fp.spherical == expect_spherical);
	std::printf("%s[%s]: spherical=%d 残差=%.6f m (%s)\n",
	            pass ? "PASS" : "FAIL", tag, fp.spherical ? 1 : 0,
	            fp.max_axis_dist_m, fp.message.c_str());
	return pass ? 0 : 1;
}
}  // namespace

int main(int argc, char ** argv)
{
	int bad = 0;
	// [1] 合成正例: j6 零偏移 → 三轴共点
	bad += check_case("合成正例(零偏移)", synthetic_urdf(0.0), true, "base", "tool");
	// [2] 合成反例: j6 垂直偏移 5cm → 非球腕 (远超 1mm 容差)
	bad += check_case("合成反例(5cm偏移)", synthetic_urdf(0.05), false, "base", "tool");
	// [3] piper 实测 (需脚本 xacro 展开 + 传 base/tip)
	if (argc > 3)
	{
		FILE * f = std::fopen(argv[1], "rb");
		if (f == nullptr)
		{
			std::printf("FAIL[piper]: 打不开 %s\n", argv[1]);
			return 1;
		}
		std::string urdf;
		char buf[4096];
		size_t n = 0;
		while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0)
		{
			urdf.append(buf, n);
		}
		std::fclose(f);
		bad += check_case("piper实测", urdf, true, argv[2], argv[3]);

		// [4] FK-IK 回代对拍 (黄金测试): 随机 q -> FK -> IK -> FK 回代必须重合
		unistackbot_algorithm::UrdfFk fk2;
		std::string msg2;
		if (!fk2.init(urdf, argv[2], argv[3], msg2))
		{
			std::printf("FAIL[回代]: UrdfFk init 失败: %s\n", msg2.c_str());
			return 1;
		}
		unistackbot_algorithm::AnalyticPiper ik;
		if (!ik.init(&fk2, msg2))
		{
			std::printf("FAIL[回代]: 求解器 init 失败: %s\n", msg2.c_str());
			return 1;
		}
		const unsigned int nj = fk2.jointCount();
		RedundancyPreference red{};   // 默认 PRESERVE
		std::srand(42u);   // 固定种子 = 可复现
		int solved = 0, multi = 0, skipped = 0;
		double worst_pos = 0.0, worst_rot = 0.0;
		const int kSamples = 200;
		for (int i = 0; i < kSamples; ++i)
		{
			std::vector<double> q(nj);
			for (unsigned int k = 0; k < nj; ++k)
			{
				const double f = std::rand() / (RAND_MAX + 1.0);
				q[k] = fk2.qMin()[k] + f * (fk2.qMax()[k] - fk2.qMin()[k]);
			}
			unistackbot_algorithm::CartesianPose t;
			if (!fk2.fk(q, t))
			{
				++skipped;
				continue;
			}
			std::vector<double> out;
			unistackbot_algorithm::DlsIkStats st;
			const auto r = ik.solve(t, q, red, out, &st,
			                        unistackbot_algorithm::SolveMode::STREAMING);
			if (r != unistackbot_interface::IkResult::OK)
			{
				if (skipped < 3)   // 首例诊断: 结果码 + ComputeFk 对拍 + 原始解数
				{
					IkReal ee[3], er[9];
					ComputeFk(q.data(), ee, er);
					std::vector<std::vector<double>> all;
					const int na = ik.solveAll(t, all);
					std::printf("诊断[样本 %d]: result=%d 目标=(%.4f,%.4f,%.4f) "
					            "ComputeFk=(%.4f,%.4f,%.4f) 原始解数=%d\n",
					            i, static_cast<int>(r), t.x, t.y, t.z,
					            ee[0], ee[1], ee[2], na);
					std::printf("  q=(");
					for (size_t k2 = 0; k2 < q.size(); ++k2) std::printf("%.3f ", q[k2]);
					std::printf(") qMin=(");
					for (size_t k2 = 0; k2 < fk2.qMin().size(); ++k2) std::printf("%.2f ", fk2.qMin()[k2]);
					std::printf(") qMax=(");
					for (size_t k2 = 0; k2 < fk2.qMax().size(); ++k2) std::printf("%.2f ", fk2.qMax()[k2]);
					std::printf(")\n  解0=(");
					if (!all.empty())
						for (size_t k2 = 0; k2 < all[0].size(); ++k2) std::printf("%.3f ", all[0][k2]);
					std::printf(")\n");
				}
				++skipped;   // FK 自产的限位内目标必可达; 解不出 = 实现缺陷
				continue;
			}
			++solved;
			unistackbot_algorithm::CartesianPose back;
			if (fk2.fk(out, back))
			{
				const double dx = back.x - t.x, dy = back.y - t.y, dz = back.z - t.z;
				const double perr = std::sqrt(dx * dx + dy * dy + dz * dz);
				double dot = back.qw * t.qw + back.qx * t.qx + back.qy * t.qy + back.qz * t.qz;
				if (dot < 0) dot = -dot;
				const double rerr = 2.0 * std::acos(std::min(1.0, dot));
				worst_pos = std::max(worst_pos, perr);
				worst_rot = std::max(worst_rot, rerr);
				// 容差 1e-4: 含规范化 URDF 的 88µm 偏差账 + double 精度
					if (perr > 1e-4 || rerr > 1e-4)
				{
					std::printf("FAIL[回代]: 样本 %d 超差 pos=%.2e rot=%.2e\n", i, perr, rerr);
					++bad;
				}
			}
			std::vector<std::vector<double>> all;
			const int na = ik.solveAll(t, all);
			if (na > 1) ++multi;
		}
		std::printf("回代: %d/%d 解出 (跳过 %d), 多解样本 %d, 最差位置误差 %.2e m / 姿态误差 %.2e rad\n",
		            solved, kSamples, skipped, multi, worst_pos, worst_rot);

		// [5] 近目标可行性探针: 零位 + 3cm 各方向 (demo_cartesian 的目标形态)
		{
			unistackbot_algorithm::CartesianPose t0;
			fk2.fk(std::vector<double>(nj, 0.0), t0);
			const char * dirs[5] = {"+x", "-x", "+y", "-y", "+z"};
			const double dv[5][3] = {{0.03, 0, 0}, {-0.03, 0, 0}, {0, 0.03, 0},
			                         {0, -0.03, 0}, {0, 0, 0.03}};
			for (int d = 0; d < 5; ++d)
			{
				unistackbot_algorithm::CartesianPose t = t0;
				t.x += dv[d][0];
				t.y += dv[d][1];
				t.z += dv[d][2];
				std::vector<std::vector<double>> all;
				const int raw = ik.solveAll(t, all);
				int inlim = 0;
				for (const auto & q : all)
				{
					bool ok = true;
					for (size_t k = 0; k < q.size() && ok; ++k)
					{
						ok = (q[k] >= fk2.qMin()[k] && q[k] <= fk2.qMax()[k]);
					}
					if (ok) ++inlim;
				}
				std::printf("探针[%s]: 原始解 %d, 限位内 %d\n", dirs[d], raw, inlim);
			}
		}
	}
	else
	{
		std::printf("WARN: 未传 piper URDF, 跳过实测用例\n");
	}
	std::printf("结果: %s\n", bad == 0 ? "全部通过" : "存在失败");
	return bad == 0 ? 0 : 1;
}
