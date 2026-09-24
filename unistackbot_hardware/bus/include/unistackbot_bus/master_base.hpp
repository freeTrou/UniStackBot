#ifndef UNISTACKBOT_BUS__MASTER_BASE_HPP_
#define UNISTACKBOT_BUS__MASTER_BASE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "unistackbot_protocol/protocol_core.hpp"
#include "unistackbot_statemachine/neutral_state.hpp"

// MasterBase —— 总线主站父类 (2026-09-24 立轴, 同 protocol/statemachine 组织:
// 父类定接口 · 子类实现 · 工厂按名创建 · 型号↔总线由机器配置显式声明)。
//
// 消费方: SystemInterface (ros2_control 适配层, 全工程唯一碰 ROS 的总线路径)。
// 交互类别 (2026-09-24 补全, 覆盖 PDO/SDO 两形):
//   ①周期交换 (PDO 形): publish_cmd/take_state —— 定槽覆盖写/取最新, RT 路径;
//   ②事务访问 (SDO 形): read_param/write_param —— 请求-应答带超时, 非 RT 慢通道
//     (协议层六组件之"④慢通道 mailbox"的父类投影);
//   ③即发命令: quick_stop (清故障类操作经 ②的事务写或将来 special 扩展)。
// 内部实现 (泵线程/时隙/transport) 对调用方零可见。
//
// 交换类型自持 (BusCommand/BusState, protocol 包 Node 类型定长数组) —— 不引用
// interface 包的 RobotCommand/RobotFeedback (ROS 包), 保持零 ROS 链:
// interface ← protocol ← statemachine ← bus ← (骨架子类)。
//
// 子类: SerialMaster (串口骨架, 切片1步4 落地并注册工厂); EthercatMaster/CanfdMaster
// 按加法纪律后续进注册表。

namespace unistackbot_bus
{

// 节点表上限 (总线一段的节点数界; 与 interface 的 kMaxJoints=16 同口径镜像)
constexpr std::size_t kMaxNodes = 16;

// 总线段装配配置 (组合根从机器配置读入; 无默认纪律——非法即 start 拒绝)
struct MasterConfig
{
	std::string protocol;    // 协议内芯名 (protocol 工厂口径)
	std::string translator;  // 状态翻译器名 (statemachine 工厂口径)
	std::string endpoint;    // 总线端点 (串口=设备路径; CAN=接口名; EC=网卡——中立载体)
	double rate_hz = 0.0;    // 总线拍频 (>0 必填)
	std::size_t node_count = 0;                       // 节点表长度
	std::uint8_t node_id[kMaxNodes] = {};             // 节点总线 ID 表 (定槽)
	double ratio[kMaxNodes] = {};                     // 每节点减速比 (>0)
};

// 一拍命令集 (按配置的节点表定槽; publish 覆盖写, 取最新语义)
struct BusCommand
{
	unistackbot_protocol::NodeCommand node[kMaxNodes] = {};
};

// 节点反馈快照 + 段级健康 (take 取最新; seq 递增判新旧)
struct BusState
{
	unistackbot_protocol::NodeFeedback node[kMaxNodes] = {};
	unistackbot_statemachine::NeutralState node_state[kMaxNodes] = {};
	std::uint64_t seq = 0;   // 快照序号 (按发布次数递增)
};

// 慢通道事务结果 (显式错误流, 零异常)
enum class ParamStatus : std::uint8_t
{
	kOk = 0,
	kUnsupported,   // 协议无慢通道能力 (如纯周期帧协议)
	kTimeout,       // 应答超时
	kError,         // 事务失败 (协议否定应答/参数拒绝等)
};

class MasterBase
{
public:
	virtual ~MasterBase() = default;

	// 生命周期: 装配配置并启动 (含线程/transport; 任一非法或打开失败 → false)
	[[nodiscard]] virtual bool start(const MasterConfig &cfg) = 0;
	// 停止并释放 (阻塞到泵线程退出; 安全语义: 帧停 → 驱动器看门狗兜底)
	virtual void stop() = 0;

	// 命令面: 覆盖写取最新 (非阻塞, 永不等待)
	virtual void publish_cmd(const BusCommand &cmd) = 0;
	// 状态面: 取最新快照 (非阻塞)。返回 true=拿到比上次新的快照。
	[[nodiscard]] virtual bool take_state(BusState &out) = 0;

	// 安全面: 急停原子旗 (下一拍生效, 各段按协议广播停)
	virtual void quick_stop() = 0;
	// 段级粗态 (worst-of 聚合: 任一节点 FAULT 即 FAULT, 余类推; 无反馈=kUnknown)
	[[nodiscard]] virtual unistackbot_statemachine::NeutralState state() const = 0;

	// ── 慢通道 (SDO 形事务访问: 参数/对象读写, 2026-09-24 用户纠偏补类别) ──
	// 语义: 请求-应答事务, **只许非 RT 上下文调用** (on_configure/诊断/恢复流程,
	// 同 wait_state 先例); 协议无此能力 → kUnsupported (unitree_im 周期帧协议即此类)。
	// key/value 为中立载体: CiA402 打包 idx<<8|subindex, Modbus=寄存器地址,
	// Dynamixel=控制表地址; 数值打包约定由协议内芯定义。
	// 消费场景: CiA402 写 0x6060 模式/读 0x6061、上电身份/参数确认、恢复流程。
	[[nodiscard]] virtual ParamStatus read_param(std::uint8_t node_id, std::uint32_t key, std::uint64_t &out_raw, int timeout_ms) = 0;
	[[nodiscard]] virtual ParamStatus write_param(std::uint8_t node_id, std::uint32_t key, std::uint64_t raw_value, int timeout_ms) = 0;
};

}  // namespace unistackbot_bus

#endif
