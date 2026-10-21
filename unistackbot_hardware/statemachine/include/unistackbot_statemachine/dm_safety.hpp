#ifndef UNISTACKBOT_STATEMACHINE__DM_SAFETY_HPP_
#define UNISTACKBOT_STATEMACHINE__DM_SAFETY_HPP_

#include "unistackbot_statemachine/node_safety.hpp"

// DmSafety —— 达妙 DM 系电机安全状态机 (第三个子类, 2026-10-21 stationlite A2)。
//
// 状态词汇翻译 (手册 V1.3/V1.4 全码表, error_raw=反馈 D0 高 4 位):
//   0 失能 → kReady        1 使能 → kEnabled
//   3/4/5 校准类 + 8-E 故障 → kFault (粘性, 清故障握手离开)
//   无独立超时位: 通讯丢失=ERR D → kFault (驱动器看门狗语义走故障码)
//
// 使能/停机语义 (与 unitree 的关键差异):
//   enable  = 独立特殊帧 (kEnable→0xFC, 帧无参数) —— 使能后第一拍 MIT 锚定
//             实测位归主站控制流 (安全怠速先例), 状态机不掺和增益。
//   disable = 失能帧 (kStop→0xFD)。**警示: 失能=三相释放无保持力, 重力臂下垂** ——
//             仅紧急停/运输态用; 运行中"停"= 持续 MIT 锚定 (断流语义归框架侧)。
//
// 工厂注册 dm4310/dm4340 两 key (与协议工厂同一把; 码表全系列同, 型号无关)。

namespace unistackbot_statemachine
{

class DmSafety final : public NodeSafety
{
public:
	void update(const unistackbot_protocol::NodeFeedback & fb) override;
	[[nodiscard]] unistackbot_protocol::NodeCommand enable_command() const override;
	[[nodiscard]] unistackbot_protocol::NodeCommand disable_command() const override;
	// 达妙覆写: 急停/驻留 = 锚定实测位 + 先验 kp/kd 阻尼 MIT (保持力, 不砸不掉);
	// 失能态下 MIT 帧被驱动器忽略 (无害), 使能态下提供受控驻留
	[[nodiscard]] unistackbot_protocol::NodeCommand safety_stop_command() const override;
	[[nodiscard]] unistackbot_protocol::NodeCommand idle_command() const override;
};

}  // namespace unistackbot_statemachine

#endif  // UNISTACKBOT_STATEMACHINE__DM_SAFETY_HPP_
