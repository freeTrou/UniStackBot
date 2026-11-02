#ifndef UNISTACKBOT_ALGORITHM__UPPER_BODY_HQP_MOS_HPP_
#define UNISTACKBOT_ALGORITHM__UPPER_BODY_HQP_MOS_HPP_

/*
 * UpperBodyHqpMos —— WholeBodySolver 契约的参考实现 (2026-10-29, 示例身份):
 * mos arm_controller WbcSolver (pinocchio+qpOASES 的双臂 HQP) 经适配层重宿主,
 * 用于验证 whole_body.hpp 两方法薄契约足以承载真 HQP 数学 —— mos 数学零改动。
 *
 * 住址: 本头为**参考实现**; 生产归宿 = 外部仓模式 (mos 侧) 或 whole_body_plugins。
 * 依赖闭包: 编译只需 vendor/mos_wbc/ 两头 (pimpl 把 pinocchio/qpOASES/yaml-cpp
 * 藏在实现侧) + Eigen; **链接**需 mos 实现体 (在 mos 仓, 或未来可选子库引入)。
 *
 * 形态: 双臂 (P20) —— mos 数学 nv/2 + 左右手槽位焊死 (适配层形态选拒显式声明);
 * 单臂 = 索引集手术后的另一实现, 契约不变 (增实现不增接口)。
 *
 * 重宿主语义决定 (适配层职责, 数学零改动):
 *   - mos 原设计**开环积分** (setCartesianTarget 内部 integrate 推进 current_q_
 *     不回锚测量); 本层改为每拍以实测重锚 = 闭环消漂。posture_reference_ 由 mos
 *     首拍自动锚定, 姿态任务语义保持。
 *   - mos setNewJoint 实为状态注入口 (名字骗人, "规划器参考"仅软停路径在用);
 *     ddq 注入侧未被 QP 消费, 本层传零。
 *
 * 实例暴露的契约缺口 (审核结论):
 *   ① **已收口 (v0.1, 用户批)**: solve 返回 WholeBodyResult 枚举; 本示例预检
 *      isCartesianTargetReachable 分类 UNREACHABLE, 预检过后仍失败 = SOLVE_FAILED
 *      (mos 返回 bool 不细分限位冲突/QP 失败——LIMIT_CONFLICT 预留给手术后实现,
 *      细分需 mos 侧失败原因出口, 与缺口② getJointNames 同族)。
 *   ② 关节**序**校验缺出口: mos 未暴露 getJointNames(), 当前依赖 yaml 与 URDF
 *      同序 (mos 侧一个名字 getter 即闭)。
 *   ③ 任务误差遥测适配层自算 (目标−当前帧), 免 mos 加误差出口。
 *
 * 规范检查命令 (g++ 直编, 零 ROS):
 *   g++ -std=c++17 -Wall -Wextra -fsyntax-only -I<本包根> -I/usr/include/eigen3 \
 *       whole_body/upper_body_hqp_mos.hpp
 */
#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <unistd.h>

#include <Eigen/Geometry>

#include "urdf_fk/urdf_fk.hpp"
#include "vendor/mos_wbc/wbc_solver.h"
#include "whole_body.hpp"

class UpperBodyHqpMos : public unistackbot_algorithm::WholeBodySolver
{
public:
	[[nodiscard]] bool init(const unistackbot_algorithm::WholeBodyConfig & config, std::string & why) override
	{
		// 形态选拒 (契约 3): mos 数学双臂焊死 —— 本实现只服务双臂形态, 不代选不猜测
		if (config.end_frames.size() != 2)
		{
			why = "upper_body_hqp(mos 版) 为双臂实现, end_frames 须恰为 2 (当前 " + std::to_string(config.end_frames.size()) + ")";
			return false;
		}
		// v0.2 字段防呆 + base_frame 校验 (契约条款"不许忽略"): 自家 UrdfFk 探针
		// base→end_frames[0] 建链, 成功 = 参考系与末端帧真实存在且连通
		if (config.base_frame.empty() || !(config.dt_s > 0.0))
		{
			why = "base_frame 与 dt_s 为必填装配参数 (base_frame 非空, dt_s > 0)";
			return false;
		}
		{
			unistackbot_algorithm::UrdfFk probe;
			if (!probe.init(config.urdf, config.base_frame, config.end_frames[0], why))
			{
				why = "base_frame/end_frames[0] 探针建链失败: " + why;
				return false;
			}
		}
		// 过渡桥 (v0.3): mos WbcSolver 构造吃文件路径, 而契约已是 URDF 字符串——落临时文件转交。
		// 手术清单 +1: WbcSolver 构造改吃字符串后此桥整体删除 (与 getJointNames/dt getter 同批)。
		const std::string mos_urdf_path = "/tmp/wb_mos_urdf_" + std::to_string(::getpid()) + ".urdf";
		{
			std::ofstream out_file(mos_urdf_path, std::ios::trunc);
			if (!out_file.is_open())
			{
				why = "URDF 临时文件写失败: " + mos_urdf_path;
				return false;
			}
			out_file << config.urdf;
		}
		// dt_s 对齐义务: mos 的步长住在 vendor yaml 的 frequency_hz (Impl 私有无 getter,
		// 缺口②同族)——当前靠 vendor yaml 与 config.dt_s 手工一致; 索引集手术后 frequency
		// 从 vendor yaml 移除, 实现直接消费 config.dt_s (两源归一)。
		// 遥测帧表 → mos 五槽位 (拓扑词住在适配层, 契约不认识)
		static const std::pair<const char *, FrameSlot> kSlots[] = {
			{"left_hand", FrameSlot::kLeftHand}, {"right_hand", FrameSlot::kRightHand},
			{"left_camera", FrameSlot::kLeftCamera}, {"right_camera", FrameSlot::kRightCamera},
			{"head_camera", FrameSlot::kHeadCamera},
		};
		for (const auto & name : config.telemetry_frames)
		{
			const auto it = std::find_if(std::begin(kSlots), std::end(kSlots),
				[&name](const auto & s) { return name == s.first; });
			if (it == std::end(kSlots))
			{
				why = "遥测帧 '" + name + "' 不在本实现的槽位表 (left/right_hand, left/right/head_camera)";
				return false;
			}
			telemetry_.emplace_back(name, it->second);
		}
		wbc_ = std::make_unique<core::WbcSolver>(mos_urdf_path, config.vendor_config_path);
		end_frames_ = config.end_frames;   // goals 按名映射左右槽的依据 (序不承诺)
		if (wbc_->getJointNum() != static_cast<int32_t>(config.control_joints.size()))
		{
			why = "URDF nq(" + std::to_string(wbc_->getJointNum()) + ") 与 control_joints 数(" +
				std::to_string(config.control_joints.size()) + ") 不符";
			return false;
		}
		// 关节序校验缺出口 (缺口 ②): 依赖 yaml 与 URDF 同序
		n_ = config.control_joints.size();
		ready_ = true;
		return true;
	}

	[[nodiscard]] unistackbot_algorithm::WholeBodyResult solve(const unistackbot_algorithm::WholeBodyInput & in,
		unistackbot_algorithm::WholeBodyOutput & out) const override
	{
		if (!ready_ || in.q.size() != n_ || in.dq.size() != n_ || in.goals.size() != 2)
		{
			return unistackbot_algorithm::WholeBodyResult::NOT_READY;
		}
		// ① 测量重锚 (开环积分 → 闭环消漂, 见头注释; 失败=维度门 → NOT_READY)
		const std::vector<double> zero_ddq(n_, 0.0);
		if (!wbc_->setNewJoint(in.q, in.dq, zero_ddq))
		{
			return unistackbot_algorithm::WholeBodyResult::NOT_READY;
		}
		// ② 目标**按名映射**左右槽 (goals 是集合, 序不承诺——按名查位; 位置映射会静默错臂,
		//    2026-10-29 合理性审核实锤修复)。缺任一必需帧 = 调用方契约违例 → NOT_READY。
		const auto left_goal = std::find_if(in.goals.begin(), in.goals.end(),
			[this](const auto & g) { return g.frame == end_frames_[0]; });
		const auto right_goal = std::find_if(in.goals.begin(), in.goals.end(),
			[this](const auto & g) { return g.frame == end_frames_[1]; });
		if (left_goal == in.goals.end() || right_goal == in.goals.end())
		{
			return unistackbot_algorithm::WholeBodyResult::NOT_READY;
		}
		// ③ 目标预检 (v0.1 分类通道): 腕心出安全可达域/目标非有限 → UNREACHABLE
		const core::SE3 left = toSe3(left_goal->target);
		const core::SE3 right = toSe3(right_goal->target);
		if (!wbc_->isCartesianTargetReachable(left, right))
		{
			return unistackbot_algorithm::WholeBodyResult::UNREACHABLE;
		}
		// ④ 解算 (mos 内部同步算完 HQP 三层): 预检过后仍失败 = 限位不可行/QP/层级违约,
		//    mos 返回 bool 不细分 → SOLVE_FAILED; mos 内部状态停在上一解, 本层不写 out (契约 1)。
		if (!wbc_->setCartesianTarget(left, right))
		{
			return unistackbot_algorithm::WholeBodyResult::SOLVE_FAILED;
		}
		// ⑤ 取解 (getControlResult = 纯缓存读; 第④步已把 HQP 三层全部算完)
		const core::ControlResult r = wbc_->getControlResult();
		// —— 以下才开始写 out (可失败步骤全部在上) ——
		out.q = toVec(r.q);
		out.dq = toVec(r.dq);
		out.ddq = toVec(r.ddq);
		out.gravity = toVec(r.gravity);
		// ⑥ 遥测帧: 本拍解后的实际位姿 (mos update() 在 setCartesianTarget 尾部已刷新)
		out.frames.clear();
		for (const auto & [name, slot] : telemetry_)
		{
			core::SE3 se3 = core::SE3::Identity();
			switch (slot)
			{
			case FrameSlot::kLeftHand: se3 = wbc_->getLeftHandPose(); break;
			case FrameSlot::kRightHand: se3 = wbc_->getRightHandPose(); break;
			case FrameSlot::kLeftCamera: se3 = wbc_->getLeftCameraPose(); break;
			case FrameSlot::kRightCamera: se3 = wbc_->getRightCameraPose(); break;
			case FrameSlot::kHeadCamera: se3 = wbc_->getHeadCameraPose(); break;
			}
			out.frames.push_back(unistackbot_algorithm::FramePose{name, toPose(se3)});
		}
		// ⑦ 任务诊断: 误差适配层自算 (目标−当前帧, 缺口 ③的替代解); 语义注: 帧取自命令后
		// 位姿 (实测+本拍Δ)——度量的是"发出的命令离目标多远", 与测量帧误差差一拍; 收敛阈定稿时入 vendor_config
		out.tasks.clear();
		constexpr double kConvTol = 1e-3;
		for (const auto & goal : in.goals)
		{
			const auto it = std::find_if(out.frames.begin(), out.frames.end(),
				[&goal](const auto & f) { return f.frame == goal.frame; });
			if (it == out.frames.end())
			{
				continue;   // 目标帧未列入遥测表 = 无从对账, 跳过 (不拒解)
			}
			const double dx = goal.target.x - it->pose.x;
			const double dy = goal.target.y - it->pose.y;
			const double dz = goal.target.z - it->pose.z;
			unistackbot_algorithm::TaskDiag d;
			d.task = goal.frame;
			d.error = std::sqrt(dx * dx + dy * dy + dz * dz);
			d.state = d.error < kConvTol ? unistackbot_algorithm::TaskState::CONVERGED
				: unistackbot_algorithm::TaskState::APPROACHING;
			out.tasks.push_back(d);
		}
		return unistackbot_algorithm::WholeBodyResult::OK;
	}

private:
	enum class FrameSlot : uint8_t
	{
		kLeftHand = 0,
		kRightHand = 1,
		kLeftCamera = 2,
		kRightCamera = 3,
		kHeadCamera = 4,
	};

	// Eigen::VectorXd → std::vector<double> (契约用中立载体, 转换住适配层)
	static std::vector<double> toVec(const core::VecX & v)
	{
		return std::vector<double>(v.data(), v.data() + v.size());
	}

	static core::SE3 toSe3(const unistackbot_algorithm::CartesianPose & p)
	{
		core::SE3 t = core::SE3::Identity();
		const Eigen::Quaterniond q(p.qw, p.qx, p.qy, p.qz);
		t.linear() = q.normalized().toRotationMatrix();
		t.translation() = Eigen::Vector3d(p.x, p.y, p.z);
		return t;
	}

	static unistackbot_algorithm::CartesianPose toPose(const core::SE3 & t)
	{
		unistackbot_algorithm::CartesianPose p;
		p.x = t.translation().x();
		p.y = t.translation().y();
		p.z = t.translation().z();
		const Eigen::Quaterniond q(t.linear());
		p.qw = q.w();
		p.qx = q.x();
		p.qy = q.y();
		p.qz = q.z();
		return p;
	}

	mutable std::unique_ptr<core::WbcSolver> wbc_;   // const solve + mutable 内芯 (IkSolver 契约 4 实例线程私有同款)
	std::vector<std::string> end_frames_;                        // goals 按名映射左右槽的依据
	std::vector<std::pair<std::string, FrameSlot>> telemetry_;   // config 名 → mos 五槽位
	bool ready_{false};
	size_t n_{0};
};

#endif  // UNISTACKBOT_ALGORITHM__UPPER_BODY_HQP_MOS_HPP_
