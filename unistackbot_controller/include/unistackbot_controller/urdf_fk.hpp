#ifndef UNISTACKBOT_CONTROLLER__URDF_FK_HPP_
#define UNISTACKBOT_CONTROLLER__URDF_FK_HPP_

#include <memory>
#include <string>
#include <vector>

#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace unistackbot_controller
{

/*
 * 笛卡尔位姿 —— 值类型, 包内自用 (interface 章程: 两包以上消费才上收)。
 */
struct CartesianPose
{
	double x{0.0}, y{0.0}, z{0.0};               // 位置 [m]
	double qw{1.0}, qx{0.0}, qy{0.0}, qz{0.0};   // 姿态四元数 (w 在前, 单位化)
};

/*
 * URDF 运动学库 —— 形态盲: 机型只是 URDF 数据, 库零机型知识。
 *
 * 能力: FK (关节角 -> 末端位姿) + 雅可比 (关节角 -> 6×n 末端速度映射)。
 * 数学: KDL (robot_state_publisher 同源 —— TF 对拍可到机器精度)。
 *
 * 用法契约:
 *   1. init(urdf_string, base_link, tip_link) 一次, 显式错误流 ([[nodiscard]] bool);
 *   2. 之后 fk()/jacobian() 只读, RT 安全 (无分配、无锁、无 IO);
 *   3. 关节数 n = 链上活动关节数, 由 init 探明 (jointCount())。
 */
class UrdfFk
{
public:
	/*
	 * 解析 URDF 并建链。
	 * urdf_string: 完整 URDF (XML 文本; 运行时经 robot_description 参数来)
	 * base / tip:  链的起止 link 名 (如 link_base -> link7)
	 * 失败原因写入 message (URDF 非法 / link 不存在 / 链含固定根异常等)。
	 */
	[[nodiscard]] bool init(
		const std::string & urdf_string, const std::string & base,
		const std::string & tip, std::string & message);

	// 链上活动关节数 (init 成功后有效)
	[[nodiscard]] unsigned int jointCount() const {return static_cast<unsigned int>(q_min_.size());}

	// FK: 关节角 (长度 = jointCount) -> 末端位姿。init 未成功返回 false。
	[[nodiscard]] bool fk(const std::vector<double> & q, CartesianPose & out) const;

	// 雅可比: 关节角 -> 6×n (上 3 行 = 线速度, 下 3 行 = 角速度), 行主序
	[[nodiscard]] bool jacobian(const std::vector<double> & q, std::vector<double> & jac_rowmajor) const;

	// 关节限位 (init 时自 URDF 提取; 长度 = jointCount)
	[[nodiscard]] const std::vector<double> & qMin() const {return q_min_;}
	[[nodiscard]] const std::vector<double> & qMax() const {return q_max_;}
	// 链序关节名 (基座->tip; fk() 的 q 向量按此顺序)
	[[nodiscard]] const std::vector<std::string> & jointNames() const {return joint_names_;}

private:
	KDL::Chain chain_;
	std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
	std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
	std::vector<double> q_min_, q_max_;   // 链序限位 (init 时自 URDF 提取)
	std::vector<std::string> joint_names_;   // 链序关节名
	bool ready_{false};
};

}  // namespace unistackbot_controller
#endif  // UNISTACKBOT_CONTROLLER__URDF_FK_HPP_
