#ifndef UNISTACKBOT_UDP_BOARD__BOARD_PROTOCOL_HPP_
#define UNISTACKBOT_UDP_BOARD__BOARD_PROTOCOL_HPP_
/*
 * Lumos UDP 电控板帧契约 (Phase 0B, 2026-10-30 动工; ODE 附录 A 归宿: mos_hal 融入 bus 第四总线)
 *
 * golden 来源 (单一事实源, 按可信度排序):
 *   1. debug/example_run_frames.md 字节实录 (mos_hal2.0 @ origin/feature/single-motor-control)
 *      —— ODE 已警告 protocol.h 与实跑有漂移, 以实录为准;
 *   2. mos_hal2.0/include/mos_hal/protocol.h (vendored 位域结构照抄于此, 布局权威 = 编译器实测)
 *
 * 帧形态 (pack(1), 小端):
 *   Control  278B = ts(1)+id(1,0x02)+bus[3]×65(臂/头)+bus_chassis(81)   [主机→板]
 *   Response 474B = ts(1)+id(1,0x01)+bus[3]×96(臂/头)+bus_chassis(184)  [板→主机]
 *   Query      2B = ts+id(0x00) —— 注: HAL main tip 用零化 Control 帧当 Query (漂移在案)
 *
 * 数值语义: 本层只做 raw 计数 ↔ 字节的搬运, 不定标 (tar_vel raw ≈ rad/s×1000 见实录;
 * 逐关节 direction/增益定标住 HAL 标定 yaml → 装配层)。
 *
 * 位域前提声明: vendored 结构体布局依赖小端 + gcc 位域分配规则, 与板固件/原 HAL 同族
 * 编译目标 (x86/AArch64 gcc) 一致——static_assert 锁全族尺寸, 黄金测试字节级对拍。
 */
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace unistackbot_udp_board
{

// ---- 帧常量 (实测/golden) ----
inline constexpr size_t kControlFrameSize = 278;
inline constexpr size_t kResponseFrameSize = 474;
inline constexpr uint8_t kControlFrameId = 0x02;
inline constexpr uint8_t kResponseFrameId = 0x01;
inline constexpr uint8_t kQueryFrameId = 0x00;
inline constexpr size_t kArmBusCount = 3;          // 臂×2 + 头 (bus[0..2])
inline constexpr size_t kArmMotorsPerBus = 8;      // _BusCmd/_BusState 每总线 8 电机
inline constexpr size_t kChassisMotorCount = 10;   // 底盘 10 电机 (4 舵 + 4 驱 + 备)
inline constexpr size_t kChassisCmdOffset = 197;   // Control 内 bus_chassis 起始 (2 + 3×65)
inline constexpr size_t kChassisCmdMotorBase = 198;
inline constexpr size_t kChassisStateOffset = 290; // Response 内 bus_chassis 起始 (2 + 3×96)

// ---- 控制模式字 (ctrl_word / status 共用语义, protocol.h 原注释) ----
inline constexpr uint8_t kCtrlWordQuery = 0x0;     // 获取数据
inline constexpr uint8_t kCtrlWordReset = 0x1;     // 复位
inline constexpr uint8_t kCtrlWordPdMit = 0x3;     // PD/MIT 控制
inline constexpr uint8_t kCtrlWordPcl = 0x4;       // PCL (夹爪)
inline constexpr uint8_t kCtrlWordAdrc = 0x5;      // ADRC
inline constexpr uint8_t kCtrlWordSpeedPi = 0x6;   // 速度 PI (底盘)
inline constexpr uint8_t kCtrlWordCatastrophe = 0x7E; // 灾难性故障

#pragma pack(push, 1)
// ---- vendored 位域结构 (照抄 protocol.h 原声明; 布局权威) ----
struct MotorCmdBits
{
	uint8_t enable : 1;
	uint16_t kd_ki : 10;
	uint16_t kp : 11;
	int16_t tar_tor : 12;
	int16_t tar_vel : 15;
	int16_t tar_pos : 15;
};
static_assert(sizeof(MotorCmdBits) == 8, "MotorCmdBits size");

struct MotorStateBits
{
	uint8_t status : 8;      // 同 ctrl_word 语义 (0x3=PD/MIT ...)
	uint8_t res : 4;
	uint8_t heart_beat : 1;
	uint8_t enable : 1;
	uint8_t error : 8;       // 位掩码: 1 模式无效 2 过压 4 欠压 8 过速 16 过温 32 过流 255 掉线
	int8_t temp : 8;
	int16_t cur_tor : 12;
	int16_t cur_vel : 15;
	int16_t cur_pos : 15;
};
static_assert(sizeof(MotorStateBits) == 9, "MotorStateBits size");

struct WrenchBits
{
	float fx, fy, fz, mx, my, mz;
};
static_assert(sizeof(WrenchBits) == 24, "WrenchBits size");

struct MotorStateChassisBits
{
	int32_t cur_vel;
	int32_t cur_pos;
	int16_t cur_tor;
	uint16_t status;
	uint32_t error;
};
static_assert(sizeof(MotorStateChassisBits) == 16, "MotorStateChassisBits size");

struct BusCmdBits
{
	uint8_t ctrl_word;
	MotorCmdBits motor[kArmMotorsPerBus];
};
static_assert(sizeof(BusCmdBits) == 65, "BusCmdBits size");

struct BusCmdChassisBits
{
	uint8_t ctrl_word;
	MotorCmdBits motor[kChassisMotorCount];
};
static_assert(sizeof(BusCmdChassisBits) == 81, "BusCmdChassisBits size");

struct BusStateBits
{
	MotorStateBits motor[kArmMotorsPerBus];
	WrenchBits wrench;
};
static_assert(sizeof(BusStateBits) == 96, "BusStateBits size");

struct BusStateChassisBits
{
	MotorStateChassisBits motor[kChassisMotorCount];
	WrenchBits wrench;
};
static_assert(sizeof(BusStateChassisBits) == 184, "BusStateChassisBits size");

struct ControlFrameBits
{
	uint8_t ts;
	uint8_t id;
	BusCmdBits bus[kArmBusCount];
	BusCmdChassisBits bus_chassis;
};
static_assert(sizeof(ControlFrameBits) == kControlFrameSize, "ControlFrameBits size");

struct ResponseFrameBits
{
	uint8_t ts;
	uint8_t id;
	BusStateBits bus[kArmBusCount];
	BusStateChassisBits bus_chassis;
};
static_assert(sizeof(ResponseFrameBits) == kResponseFrameSize, "ResponseFrameBits size");
#pragma pack(pop)

// ---- 纯函数编解码 (对拍 fake_motor.py 的小端整帧解释; 失败不改 out, 显式错误流) ----
// 注: 本族只覆盖 Control/Response 两帧; Query (2B, id=0x00) 与 HAL main tip 的
// "零化 Control 当 Query" 漂移形态归主站批次处理 (届时加 encodeQuery/守门)。
// 位域字段值域: 15 位有符号 [-16384, 16383] / 12 位 [-2048, 2047] / kp 11 位 [0,2047] /
// kd_ki 10 位 [0,1023]——本层纯搬运不做范围检查 (越界赋值按 C++ 位域语义截断, 调用方自律)。

// 命令帧: 场景值 → 字节 (len 必须为 kControlFrameSize; id 写 kControlFrameId)
[[nodiscard]] bool encodeControl(const ControlFrameBits & in, uint8_t * out, size_t len);
// 命令帧字节 → 场景值 (长度/帧头 id 校验, 任一不符返回 false)
[[nodiscard]] bool decodeControl(const uint8_t * buf, size_t len, ControlFrameBits & out);
// 反馈帧同款
[[nodiscard]] bool encodeResponse(const ResponseFrameBits & in, uint8_t * out, size_t len);
[[nodiscard]] bool decodeResponse(const uint8_t * buf, size_t len, ResponseFrameBits & out);

// 便捷访问 (golden 实录的偏移口径, 供测试/FakeBoard 核对用; 越界由 len 校验前置保证)
[[nodiscard]] const MotorCmdBits * chassisCmdMotor(const ControlFrameBits & f, size_t i);
[[nodiscard]] const MotorStateChassisBits * chassisStateMotor(const ResponseFrameBits & f, size_t i);

}  // namespace unistackbot_udp_board
#endif  // UNISTACKBOT_UDP_BOARD__BOARD_PROTOCOL_HPP_
