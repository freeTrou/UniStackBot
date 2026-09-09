#ifndef UNISTACKBOT_SIM_CONTROL__SIM_BACKEND_HPP_
#define UNISTACKBOT_SIM_CONTROL__SIM_BACKEND_HPP_

#include <string>
#include <vector>

namespace unistackbot_sim_control
{

//关节元数据: 由插件从 <ros2_control> 块解析, 所有后端共用
struct JointMeta
{
	std::string name;                // 关节名 (与 URDF 一致, 索引/校验用)
	double min{0.0};                 // 位置下限 (rad)
	double max{0.0};                 // 位置上限 (rad)
	double max_velocity{0.0};        // rad/s, 独立关节的执行器速度上限
	int mimic_source{-1};            // -1 = 独立关节; 否则源关节索引
	double mimic_multiplier{1.0};    // mimic 变换: 位置 = multiplier * 源关节 + offset
	double mimic_offset{0.0};        // mimic 变换偏置 (rad)

	bool is_mimic() const
	{
		return mimic_source >= 0;
	}
};

/*
 * 仿真后端接口: 统一仿真控制层的"对内分类"契约。
 * 新仿真平台接入 = 实现本接口 + 在工厂 (sim_backend_factory.hpp) 登记一个名字。
 *
 * 后端分两类, 对宿主 SimControlHardware 完全透明 (同一套调用, 零特判):
 *   同步后端 —— plant 是 step() 里的一次函数调用 (确定性回归; 生命周期用默认空实现)
 *   异步后端 —— plant 是自带节拍的线程 (总线时序; 覆写 activate/deactivate 起停线程)
 *
 * 语义契约 (接口层定义, 所有后端同义):
 *   init        装配期一次性; 失败即拒绝加载 (装配期暴露, 不留运行期惊喜)
 *   activate    on_activate 转发; 异步后端在此起 plant, 失败则 RT 循环不启动
 *   step        read() 内调用, 收 plant 状态并推进 (对应真机的收总线/读编码器);
 *               dt = CM 周期, integrate=false 表示冻结推进但 mimic 派生量仍须刷新;
 *               cmd/state 数组归宿主所有, 后端只填不持有
 *   transmit    write() 内调用, 把宿主 cmd 下发 plant (对应真机的发总线帧);
 *               同步后端继承默认空实现 (命令已在 step 中消化)
 *   requestState 状态直写 (瞬移/回零) 下发 plant; 同步后端默认空 (宿主数组即 plant 状态)
 *   deactivate  on_deactivate 转发; 异步后端在此停线程 (join 必须有界)
 *
 * 异步后端的内部交换原语约定 (对宿主不可见):
 *   cmd (宿主→plant) 无锁单发或覆盖写取最新; state (plant→宿主) 覆盖写取最新快照
 */
class SimBackend
{
public:
	virtual ~SimBackend() = default;

	//后端初始化 (限位/mimic 等元数据由插件解析好传入)
	virtual bool init(const std::vector<JointMeta> & joints, std::string & message) = 0;

	//后端名 (诊断/日志标识)
	virtual const std::string & name() const = 0;

	//起 plant (异步后端覆写: 启动 plant 线程; 同步后端继承默认 = 无事可做)
	[[nodiscard]] virtual bool activate()
	{
		return true;
	}

	//停 plant (异步后端覆写: 降旗标 + 有界 join; 同步后端继承默认 = 无事可做)
	virtual void deactivate() {}

	//发命令 (宿主 write() 转发): 异步后端覆写为 cmd 下发 plant (对应真机的发总线帧)
	//同步后端继承默认 = 无事可做 (命令已在 step 中消化)
	virtual void transmit(const std::vector<double> & /*cmd_position*/) {}

	//状态直写 (瞬移/回零): 异步后端覆写为下发 plant; 同步后端继承默认 (宿主数组即 plant 状态)
	virtual void requestState(const std::vector<double> & /*state_position*/, const std::vector<double> & /*state_velocity*/) {}

	/*
	 * 推进仿真一个控制周期 (宿主 read() 内调用, RT 线程)。
	 * 同步后端: 就地积分; 异步后端: 取回 plant 最新快照填入 state (命令经 transmit 发布)。
	 * integrate=false 时只刷新派生量 (mimic 关节), 用于 pause 冻结与状态瞬移后的同步。
	 */
	virtual void step(
		const std::vector<double> & cmd_position,
		std::vector<double> & state_position,
		std::vector<double> & state_velocity,
		double dt, bool integrate) = 0;
};

}  // namespace unistackbot_sim_control
#endif  // UNISTACKBOT_SIM_CONTROL__SIM_BACKEND_HPP_
