#ifndef UNISTACKBOT_PROTOCOL__DM_MOTOR_CORE_HPP_
#define UNISTACKBOT_PROTOCOL__DM_MOTOR_CORE_HPP_

#include <cstddef>
#include <cstdint>

#include "unistackbot_protocol/protocol_core.hpp"

// dm_motor —— 达妙 DM 系关节电机协议内芯 (第二个实现, stationlite 真机接入 A1)。
//
// 依据: DM-J4310-2EC V1.2 说明书 V1.4 + DM-J4340-2EC V1.1 说明书 V1.3 (官方仓
// gitee.com/kit-miao/damiao) + startouchlib 源码逐行核实 + can0 回环 mcap 实测,
// 三方闭环。规格全档见 arms/stationlite/damiao_protocol.md。
//
// 帧格式 (经典 CAN 1Mbps, 8B 数据段; **CAN ID 住帧头由主站路由, 数据段不含 ID**):
//   MIT 控制 8B (CAN ID=send): p[16] v[12] kp[12] kd[12] tau[12] 位拼接
//   反馈     8B (CAN ID=recv): D0=(ERR<<4)|ID, POS[16] VEL[12] T[12], T_MOS, T_Rotor (℃直读)
//   特殊命令 8B (CAN ID=send): FF×7 + cmd —— FC 使能 / FD 失能 / FB 清故障
// 定点 (全幅编码, 分母 (2^bits)-1 非 2^bits):
//   p∈[-pMax,pMax]16 · v∈[-vMax,vMax]12 · kp∈[0,500]12 · kd∈[0,5]12 · tau∈[-tMax,tMax]12
//   反馈量程同上 (POS/VEL/T 用同表)。
//
// **一个子类覆盖全电机型号** (2026-10-21 用户裁决: 型号对臂是同一协议):
//   DmMotorType 只是限值档位 (pMax/vMax/tMax), 经构造注入, 不是协议分叉。
//   工厂 dm4310/dm4340 两 key = 同一 DmMotorCore 类的两个档位入口。
//
// 范围裁决 (damiao_protocol.md §11 第一版 = 用得到+已验证):
//   实现: MIT 帧/反馈帧/使能/失能/清故障帧。暂缓: POS_VEL/VEL/POS_FORCE 帧、
//   参数帧 0x33/0x55、存储帧 0xAA、设零位 0xFE (标定工具另验)。
//
// ratio 槽: 达妙帧上即输出/电机同域 (减速比内含于量程), 恒等 —— encode/decode
//   要求 ratio==1.0, 非 1 返回失败 (配置错误的 fail-fast, 不静默忽略)。
//
// 域约定: 帧上 q/dq/tau = 电机域原样; motor_signs (±1 双向同乘) 归上层机型
//   配置, 不进协议 (2026-10-21 冻结)。
//
// 测试 golden: test/test_dm_motor.cpp 以 startouchlib codec 为甲骨文逐字节对拍
//   (test/oracle_startouch/, Apache-2.0 提取副本)。

namespace unistackbot_protocol
{
namespace dm_motor
{

constexpr std::size_t kFrameLen = 8;   // 命令/反馈/特殊帧通用长度

// 电机型号 = 限值档位 (MIT 定点量程; 手册+SDK 双源, 语义差异见 damiao_protocol.md §3:
// tMax 是协议映射上界, 物理峰值与整机安全档另见手册/URDF effort)
enum class DmMotorType : std::uint8_t
{
	kDm4310 = 0,   // stationlite j4-j6 + 夹爪; pMax 12.5, vMax 30, tMax 10
	kDm4340 = 1,   // stationlite j1-j3;      pMax 12.5, vMax 8,  tMax 28
};

struct LimitParam
{
	double pmax;
	double vmax;
	double tmax;
};

// 限值表 (dm_motor_constants 同源, 逐值双源核对)
[[nodiscard]] LimitParam limits(DmMotorType type);

// MIT 增益界 (协议常量, 手册+SDK 双证)
constexpr double kKpMax = 500.0;
constexpr double kKdMax = 5.0;

// 增益先验 (2026-10-21, 宇树 kPriorKp 同款先例): 真机整机档案保守档 (can0 mcap
// j5/j6=45/2 —— 全臂最弱关节的安全值)。DmSafety 的 idle/safety_stop 锚定形态用;
// 真机机型 yaml 的逐关节增益是终值 (命令面 MIT kp/kd 可运行时覆盖), 先验仅兜底。
constexpr double kPriorKp = 45.0;
constexpr double kPriorKd = 2.0;

// 纯函数: 命令 → 8B 帧。mode 分派: kRun→MIT / kEnable→使能 / kStop→失能 /
// kClear→清故障。失败 (0): buf 不足 / ratio≠1 / type 非法 / MIT 帧任一值非有限。
// node_id 不入帧 (CAN ID 住帧头) —— 参数仅为接口对齐, 不校验。
[[nodiscard]] std::size_t encodeFrame(std::uint8_t *buf, std::size_t cap, const NodeCommand &cmd, std::uint8_t node_id, double ratio, DmMotorType type);

// 纯函数: 8B 帧 → 反馈。失败: len≠8 / ratio≠1 / type 非法。
// 输出域: position/speed/tau=电机域, temp 直读℃, error_raw=ERR 半字节 (0/1/3-5/8-E),
// state_raw=D0 原样 (ID|ERR), node_id=D0 低 4 位; mode 无协议信息保持 kStop。
[[nodiscard]] bool decodeFrame(const std::uint8_t *frame, std::size_t len, NodeFeedback &fb, double ratio, DmMotorType type);

/*
 * 镜像方向 (2026-10-27, can1 虚拟电机/录制分析消费; 主站永不经 ProtocolCore 调用,
 * 故为纯函数不进接口)
 * 纯函数: 8B 命令帧 → 命令。MIT 帧解位拼接; FF×7+FC/FD/FB 还原 kEnable/kStop/kClear。
 * 失败: buf 空 / len≠8 / ratio≠1 / type 非法。node_id 不入帧 (CAN ID 住帧头)。
 */
[[nodiscard]] bool unpackCommand(const std::uint8_t *frame, std::size_t len, NodeCommand &cmd, double ratio, DmMotorType type);

/*
 * 纯函数: 反馈 → 8B 帧 (构造 D0=(ERR<<4)|ID + POS/VEL/T + 双温度; 温度截 int8/uint8)。
 * 失败: buf 空 / cap<8 / ratio≠1 / type 非法 / 任一值非有限。
 */
[[nodiscard]] std::size_t packFeedback(std::uint8_t *buf, std::size_t cap, const NodeFeedback &fb, double ratio, DmMotorType type);

}  // namespace dm_motor

// ProtocolCore 的达妙实现 (一个子类, 型号=构造档位; 泵经接口多态调用, 测试直打纯函数)。
class DmMotorCore final : public ProtocolCore
{
public:
	// 型号必填 (无默认纪律: 量程即协议知识, 缺配置 fail-fast)
	explicit DmMotorCore(dm_motor::DmMotorType type);

	[[nodiscard]] ProtocolKind kind() const override { return ProtocolKind::kActuator; }
	[[nodiscard]] bool ratio_valid(double ratio) const override { return ratio == 1.0; }
	[[nodiscard]] bool id_valid(std::uint8_t node_id) const override { return node_id <= 0x7Fu; }   // CAN 11bit 实用域
	[[nodiscard]] FrameSpec framespec() const override;
	[[nodiscard]] std::size_t encode(std::uint8_t *buf, std::size_t cap, const NodeCommand &cmd, std::uint8_t node_id, double ratio) const override;
	[[nodiscard]] bool decode(const std::uint8_t *frame, std::size_t len, NodeFeedback &fb, double ratio) const override;

private:
	dm_motor::DmMotorType type_;
};

}  // namespace unistackbot_protocol

#endif
