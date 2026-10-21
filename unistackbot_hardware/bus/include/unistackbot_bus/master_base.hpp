#ifndef UNISTACKBOT_BUS__MASTER_BASE_HPP_
#define UNISTACKBOT_BUS__MASTER_BASE_HPP_

#include <cstddef>
#include <memory>
#include <cstdint>
#include <string>
#include <vector>

#include "unistackbot_protocol/protocol_core.hpp"
#include "unistackbot_statemachine/neutral_state.hpp"

// MasterBase —— 总线主站父类 (2026-09-24 立轴, 同 protocol/statemachine 组织:
// 父类定接口 · 子类实现 · 工厂按名创建 · 型号↔总线由机器配置显式声明)。
//
// **组合模型 (2026-10-21 定稿, 达妙混型触发)**: 完整机器人 = 单/多总线 × 多协议组合
// × 单/多状态机 × 节点表 —— 协议内芯与状态机统一为总线内部组件, start 期 per-node
// 经工厂自建 (2×N 实例, 循环期零新建); set_protocol 段级注入式退役。两类节点 (协议
// 自声明 kind): 电机=命令+反馈+状态机必填; 传感器=只读无状态机必空 (实现挂起)。
//
// **容量动态化 (2026-10-21 二次裁决, 方案 R-a)**: 节点表/交换载荷全运行期
// (std::vector, 生命周期边界分配, RT 循环零分配) —— kMaxNodes 容量常量退役。
// 交换 = DynamicSpLatest (unistackbot_common, 装配期定容双缓冲 seqlock)。
// 保留定长的只有真格式规格层: 线上协议帧 (各内芯) 与 ROS msg (kMaxJoints,
// interface 包裁决在案)。EC 形态 (超长段) 经 ecrt domain 自有交换, 不经本层。
// 详见 docs/architecture/real_hardware_architecture.md §1A。
//
// 消费方: SystemInterface (ros2_control 适配层, 全工程唯一碰 ROS 的总线路径)。
// 交互类别 (2026-09-24 补全, 覆盖 PDO/SDO 两形):
//   ①周期交换 (PDO 形): publish_cmd/take_state —— 覆盖写/取最新, RT 路径;
//   ②事务访问 (SDO 形): read_param/write_param —— 请求-应答带超时, 非 RT 慢通道;
//   ③即发命令: quick_stop / enable / disable / fault_clear (意图三件套+急停)。
// 内部实现 (泵线程/时隙/transport) 对调用方零可见。
//
// 交换类型自持 (NodeCommand/NodeFeedback/NodeSnapshot, protocol 包类型) —— 不引用
// interface 包 (ROS 包), 保持零 ROS 链:
// interface ← protocol ← statemachine ← bus ← (骨架子类)。
//
// 子类: SerialMaster (已注册); EthercatMaster/CanfdMaster 按加法纪律进注册表。

namespace unistackbot_bus
{

// 节点规格 (组合模型: 节点 = 第一公民; 数量运行期 = 实际电机/传感器表)
struct NodeSpec
{
	std::uint8_t id = 0;          // 节点总线 ID
	std::string protocol;         // 协议名 (空 → 回退段级 MasterConfig::protocol)
	std::string statemachine;     // 状态机名 (空 → 回退段级; 传感器节点回退后须为空)
	double ratio = 0.0;           // 减速比 (语义经 ProtocolCore::ratio_valid 校验)
};

// 总线段装配配置 (组合根从机器配置读入; 机型形态见 stationlite_buses.yaml 先例)
// 名字解析四规则 (fail-fast): 节点级非空→覆写 / 空→回退段级 / 两者皆空→start 拒 /
// 工厂未知名→拒并列可用名。kind 交叉校验: 执行器协议节点 statemachine 必填,
// 传感器协议节点必空。拒绝诊断必须带节点索引与总线 ID。装配实现住各主站 start
// (私有)——提炼共享挂起, 触发器=第二个主站落地 (最小集纪律)。
struct MasterConfig
{
	// ── 段级拓扑 (endpoint/rate 必填; 两名为节点级缺省回退的简写) ──
	std::string endpoint;    // 总线端点 (串口=设备路径; CAN=接口名; EC=网卡——中立载体)
	double rate_hz = 0.0;    // 总线拍频 (>0 必填; 段独立, 与 update_rate 解耦)
	std::string protocol;    // 段级协议名 (工厂 key; 节点级空时回退至此)
	std::string statemachine;   // 段级状态机名 (同上回退)

	// ── 节点表 (数量运行期 = 实际电机/传感器; 无容量上限——交换层装配期定容) ──
	std::vector<NodeSpec> nodes;

	// ── 调优 (有默认, 部署侧覆盖) ──
	int baudrate = 6000000;              // 串口波特率 (EC/CAN 忽略或由协议定)
	int thread_priority = 85;            // 泵线程 SCHED_FIFO 优先级 (调度阶梯: EC95>CAN90>串口85)
	int cpu_affinity = 2;                // 泵线程绑核 (总线核; -1=不绑)
	std::uint64_t stale_cycles = 50;     // 无反馈判 Unknown 的拍数 (~100ms@500Hz)
};

// 节点快照 (反馈 + 粗态; trivially copyable, DynamicSpLatest 元素)
struct NodeSnapshot
{
	unistackbot_protocol::NodeFeedback fb;
	unistackbot_statemachine::NeutralState state = unistackbot_statemachine::NeutralState::kUnknown;
};

// 一拍命令载荷 (DynamicSpLatest 元素; 覆盖写, 取最新语义)
using CmdFrame = std::vector<unistackbot_protocol::NodeCommand>;
// 一拍状态载荷 (DynamicSpLatest 元素; 顺序 = cfg.nodes 序)
using StateFrame = std::vector<NodeSnapshot>;

// 慢通道事务结果 (显式错误流, 零异常)
enum class ParamStatus : std::uint8_t
{
	kOk = 0,
	kUnsupported,   // 协议无慢通道能力 (纯周期帧协议, 如 unitree_im/dm)
	kTimeout,       // 应答超时
	kError,         // 事务失败 (协议否定应答/参数拒绝等)
};

class MasterBase
{
public:
	virtual ~MasterBase() = default;

	// 生命周期: 装配配置并启动 (组合模型: 协议内芯与状态机 start 期按节点表
	// per-node 经工厂自建; 名字解析/交叉校验/诊断纪律见 MasterConfig 注释。
	// 任一非法 → 清理已建实例并返 false)
	[[nodiscard]] virtual bool start(const MasterConfig &cfg) = 0;
	// 停止并释放 (阻塞到泵线程退出; 安全语义: 帧停 → 驱动器看门狗兜底)
	virtual void stop() = 0;

	// 命令面: 覆盖写取最新 (非阻塞, 永不等待)。n 必须 == 启动段节点数 (不符=调用方 bug,
	// 静默忽略——交换层尺寸契约)。
	virtual void publish_cmd(const unistackbot_protocol::NodeCommand *cmds, std::size_t n) = 0;
	void publish_cmd(const std::vector<unistackbot_protocol::NodeCommand> &cmds)
	{
		publish_cmd(cmds.data(), cmds.size());
	}
	// 状态面: 取最新快照 (非阻塞)。out 预分配 (resize ≥ 段节点数, 交换层按位填充)。
	// 返回 true = out 为比上次新的完整快照。
	[[nodiscard]] virtual bool take_state(std::vector<NodeSnapshot> &out) = 0;

	// 安全面: 急停原子旗 (下一拍生效, 各段按协议广播停)
	virtual void quick_stop() = 0;
	// ── 意图接口三件套 (2026-10-21 补齐 enable/disable; 语义边界) ──
	// enable/disable = **意图请求** (异步下一拍生效): 泵经状态机守卫 (仅 READY 可
	// 使能, FAULT 拒且意图失效) → 发协议使能/断能帧 → 反馈确认后请求自清。
	// quick_stop 与它们的区别: 急停是**原子旗**不等状态机全段立即 (压一切, 含意图);
	// 意图是协商流有守卫有确认。publish_cmd 的 mode 字段 = 运行中数据流, 不作使能的
	// 隐蔽通道 (职责分离)。disable = 操作员显式断能 (运输/维护, 失能无保持力警示)。
	virtual void enable(std::uint8_t node_id) = 0;
	virtual void disable(std::uint8_t node_id) = 0;
	// 安全面: 清故障请求旗 (全节点; 0xFB → error 持续 0 → confirm 握手, FAULT 粘性)
	virtual void fault_clear() = 0;
	// 段级粗态 (worst-of 聚合: 任一节点 FAULT 即 FAULT; 无反馈=kUnknown)
	[[nodiscard]] virtual unistackbot_statemachine::NeutralState state() const = 0;

	// ── 慢通道 (SDO 形事务访问, 2026-09-24 补类别; 只许非 RT 上下文调用) ──
	// 协议无此能力 → kUnsupported。参数帧组装严禁在主站内——进 ProtocolCore 加法
	// (挂账: 触发器=第一个真实现, 达妙 0x33/0x55 或 CiA402 SDO)。
	[[nodiscard]] virtual ParamStatus read_param(std::uint8_t node_id, std::uint32_t key, std::uint64_t &out_raw, int timeout_ms) = 0;
	[[nodiscard]] virtual ParamStatus write_param(std::uint8_t node_id, std::uint32_t key, std::uint64_t raw_value, int timeout_ms) = 0;
};

}  // namespace unistackbot_bus

#endif
