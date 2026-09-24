#include "urdf_fk/urdf_fk.hpp"

#include <kdl_parser/kdl_parser.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <map>
#include <string>

namespace unistackbot_algorithm
{

namespace
{

// KDL 旋转 -> 单位四元数 (w 在前; Shepperd 法, 数值稳定不依赖固定轴序)
void rotationToQuat(const KDL::Rotation & R, CartesianPose & p)
{
	const double tr = R(0, 0) + R(1, 1) + R(2, 2);
	if (tr > 0.0)
	{
		const double s = 0.5 / std::sqrt(tr + 1.0);
		p.qw = 0.25 / s;
		p.qx = (R(2, 1) - R(1, 2)) * s;
		p.qy = (R(0, 2) - R(2, 0)) * s;
		p.qz = (R(1, 0) - R(0, 1)) * s;
	}
	else if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2))
	{
		const double s = 2.0 * std::sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
		p.qw = (R(2, 1) - R(1, 2)) / s;
		p.qx = 0.25 * s;
		p.qy = (R(0, 1) + R(1, 0)) / s;
		p.qz = (R(0, 2) + R(2, 0)) / s;
	}
	else if (R(1, 1) > R(2, 2))
	{
		const double s = 2.0 * std::sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
		p.qw = (R(0, 2) - R(2, 0)) / s;
		p.qx = (R(0, 1) + R(1, 0)) / s;
		p.qy = 0.25 * s;
		p.qz = (R(1, 2) + R(2, 1)) / s;
	}
	else
	{
		const double s = 2.0 * std::sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
		p.qw = (R(1, 0) - R(0, 1)) / s;
		p.qx = (R(0, 2) + R(2, 0)) / s;
		p.qy = (R(1, 2) + R(2, 1)) / s;
		p.qz = 0.25 * s;
	}
}

}  // namespace

bool UrdfFk::init(
	const std::string & urdf_string, const std::string & base,
	const std::string & tip, std::string & message)
{
	ready_ = false;
	joint_names_.clear();
	q_min_.clear();
	q_max_.clear();

	// 建树/建链交给 kdl_parser —— 坐标系语义的既有事实源 (与 robot_state_publisher
	// 同源, TF 对拍交叉验证的本意)。手搓链在 KDL 段坐标系语义上踩过实雷: Joint origin
	// 双重平移, 症状是"q=0 全对、非 0 全错" (2026-09-17)。kdl_parser 仅依赖
	// urdfdom + orocos_kdl, 无 roscpp 包袱, "纯库"定位成立。
	KDL::Tree tree;
	if (!kdl_parser::treeFromString(urdf_string, tree))
	{
		message = "URDF 解析/建树失败 (kdl_parser)";
		return false;
	}
	if (!tree.getChain(base, tip, chain_))
	{
		message = "建链失败: " + base + " -> " + tip + " (link 名不存在或顺序颠倒)";
		return false;
	}

	// 限位与关节名: 按链序, 用 urdfdom 原生解析按关节名查 (kdl_parser 不携带限位)
	const auto model = urdf::parseURDF(urdf_string);
	if (!model)
	{
		message = "URDF 解析失败 (urdfdom)";
		return false;
	}
	const unsigned int n = chain_.getNrOfJoints();
	q_min_.reserve(n);
	q_max_.reserve(n);
	joint_names_.reserve(n);
	for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i)
	{
		const auto & joint = chain_.getSegment(i).getJoint();
		if (joint.getType() == KDL::Joint::None)
		{
			continue;
		}
		const auto it = model->joints_.find(joint.getName());
		if (it == model->joints_.end() || !it->second->limits)
		{
			message = "链关节在 URDF 中缺限位: " + joint.getName();
			return false;
		}
		joint_names_.push_back(joint.getName());
		q_min_.push_back(it->second->limits->lower);
		q_max_.push_back(it->second->limits->upper);
	}
	if (n == 0)
	{
		message = "链内无活动关节: " + base + " -> " + tip;
		return false;
	}

	// 链序关节轴 (base 系, 零位): 沿父链累计 origin 变换; 轴 = 累计旋转 * 关节 axis
	// (urdf 语义: axis 表述在关节系)。结构指纹判定用 (球腕三轴共点等)。
	axes_.clear();
	axes_.reserve(n);
	std::map<std::string, urdf::JointSharedPtr> joint_by_child;
	for (const auto & kv : model->joints_)
	{
		joint_by_child[kv.second->child_link_name] = kv.second;
	}
	for (const auto & name : joint_names_)
	{
		const auto it = model->joints_.find(name);
		if (it == model->joints_.end())
		{
			message = "链关节不在 URDF: " + name;
			return false;
		}
		const auto & j = it->second;
		std::vector<urdf::JointSharedPtr> up;   // 父链: j.parent_link -> base (自上而下回放)
		std::string cur = j->parent_link_name;
		while (cur != base)
		{
			const auto pit = joint_by_child.find(cur);
			if (pit == joint_by_child.end())
			{
				message = "URDF 链断裂: " + cur;
				return false;
			}
			up.push_back(pit->second);
			cur = pit->second->parent_link_name;
		}
		KDL::Frame w = KDL::Frame::Identity();
		for (auto rit = up.rbegin(); rit != up.rend(); ++rit)
		{
			const auto & t = (*rit)->parent_to_joint_origin_transform;
			w = w * KDL::Frame(
				KDL::Rotation::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
				KDL::Vector(t.position.x, t.position.y, t.position.z));
		}
		const auto & t = j->parent_to_joint_origin_transform;
		const KDL::Frame jf = w * KDL::Frame(
			KDL::Rotation::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
			KDL::Vector(t.position.x, t.position.y, t.position.z));
		const KDL::Vector d = jf.M * KDL::Vector(j->axis.x, j->axis.y, j->axis.z);
		axes_.push_back(JointAxis{jf.p.x(), jf.p.y(), jf.p.z(), d.x(), d.y(), d.z()});
	}

	// 链长上界: 相邻段原点距离累加 (IK 几何预检用, 保守粗上界非精确工作空间)
	max_reach_ = 0.0;
	for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i)
	{
		max_reach_ += chain_.getSegment(i).getFrameToTip().p.Norm();
	}

	fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
	jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);

	// 冒烟: 零位自算一次, 求解器与链真正可用才宣布就绪 (装配期失败显式化)
	KDL::JntArray q0(n);
	KDL::Frame f;
	if (fk_solver_->JntToCart(q0, f) != KDL::SolverI::E_NOERROR)
	{
		message = "FK 冒烟失败: 零位正解报错";
		return false;
	}
	message = "URDF 运动学链就绪: " + base + " -> " + tip + " (" + std::to_string(n) + " 关节)";
	ready_ = true;
	return true;
}

bool UrdfFk::fk(const std::vector<double> & q, CartesianPose & out) const
{
	Scratch scratch;
	return fk(q, out, scratch);
}

bool UrdfFk::fk(const std::vector<double> & q, CartesianPose & out, Scratch & scratch) const
{
	if (!ready_ || q.size() != jointCount())
	{
		return false;
	}
	if (static_cast<std::size_t>(scratch.q.rows()) != q.size())
	{
		scratch.q.resize(static_cast<unsigned int>(q.size()));
	}
	for (unsigned int i = 0; i < q.size(); ++i)
	{
		scratch.q(i) = q[i];
	}
	KDL::Frame f;
	if (fk_solver_->JntToCart(scratch.q, f) != KDL::SolverI::E_NOERROR)
	{
		return false;
	}
	out.x = f.p.x();
	out.y = f.p.y();
	out.z = f.p.z();
	rotationToQuat(f.M, out);
	return true;
}

bool UrdfFk::jacobian(const std::vector<double> & q, std::vector<double> & jac_rowmajor) const
{
	Scratch scratch;
	return jacobian(q, jac_rowmajor, scratch);
}

bool UrdfFk::jacobian(
	const std::vector<double> & q, std::vector<double> & jac_rowmajor, Scratch & scratch) const
{
	if (!ready_ || q.size() != jointCount())
	{
		return false;
	}
	if (static_cast<std::size_t>(scratch.q.rows()) != q.size())
	{
		scratch.q.resize(static_cast<unsigned int>(q.size()));
	}
	for (unsigned int i = 0; i < q.size(); ++i)
	{
		scratch.q(i) = q[i];
	}
	if (static_cast<std::size_t>(scratch.jac.columns()) != q.size())
	{
		scratch.jac.resize(static_cast<unsigned int>(q.size()));
	}
	if (jac_solver_->JntToJac(scratch.q, scratch.jac) != KDL::SolverI::E_NOERROR)
	{
		return false;
	}
	jac_rowmajor.assign(6 * q.size(), 0.0);
	for (unsigned int r = 0; r < 6; ++r)
	{
		for (unsigned int c = 0; c < q.size(); ++c)
		{
			jac_rowmajor[r * q.size() + c] = scratch.jac(r, c);
		}
	}
	return true;
}

bool UrdfFk::jointAxesAtZero(std::vector<JointAxis> & out) const
{
	if (!ready_)
	{
		return false;
	}
	out = axes_;
	return out.size() == joint_names_.size();
}

}  // namespace unistackbot_algorithm
