#include "unistackbot_statemachine/dm_safety.hpp"

#include "unistackbot_protocol/dm_motor_core.hpp"

namespace unistackbot_statemachine
{

namespace
{

// 手册 V1.3/V1.4 全码表: 0 失能 / 1 使能为正常态; 3-5 校准类 + 8-E 故障。
// err 语义: 0/1 非故障, 其余全部 FAULT (2/6/7 未定义按故障处理——保守侧)。
[[nodiscard]] bool isFaultCode(std::uint32_t err)
{
	return err != 0u && err != 1u;
}

}  // namespace

void DmSafety::update(const unistackbot_protocol::NodeFeedback & fb)
{
	last_fb_ = fb;

	// ── 标准状态机: switch(当前态) × 输入事件 → 下一态 (UnitreeIm 同款纪律:
	// 每 case 明确合法迁移, 非法不发生; FAULT 粘性, 仅 confirm 握手离开) ──
	switch (state_)
	{
	case NeutralState::kUnknown:
		if (isFaultCode(fb.error_raw))
		{
			state_ = NeutralState::kFault;
			enable_req_ = false;
		}
		else if (fb.error_raw == 1u)
		{
			state_ = NeutralState::kEnabled;
		}
		else
		{
			state_ = NeutralState::kReady;
		}
		break;

	case NeutralState::kReady:
		if (isFaultCode(fb.error_raw))
		{
			state_ = NeutralState::kFault;
			enable_req_ = false;
		}
		else if (fb.error_raw == 1u)
		{
			state_ = NeutralState::kEnabled;
		}
		// err=0 无异常 → 保持 kReady
		break;

	case NeutralState::kEnabled:
		if (isFaultCode(fb.error_raw))
		{
			state_ = NeutralState::kFault;
			enable_req_ = false;
		}
		else if (fb.error_raw == 0u)
		{
			state_ = NeutralState::kReady;
		}
		// err=1 无异常 → 保持 kEnabled
		break;

	// kQuickStop: 达妙无独立超时位 (通讯丢失=ERR D → FAULT), 本态不可达——
	// 保持 switch 完整性, 语义同 Ready 处理
	case NeutralState::kQuickStop:
		if (isFaultCode(fb.error_raw))
		{
			state_ = NeutralState::kFault;
			enable_req_ = false;
		}
		else if (fb.error_raw == 0u)
		{
			state_ = NeutralState::kReady;
		}
		break;

	// kFault: 粘性——间歇性故障不自动恢复 (防复撞)。
	// 离开路径: request_fault_clear() → 泵发 0xFB → error 持续为 0 → confirm_fault_cleared()
	case NeutralState::kFault:
		break;

	default:
		break;
	}

	// 请求生命周期机器自持 (CiA402 同款纪律): 命令被设备确认即清——达妙确认信号=err
	// (使能帧生效→err=1; 失能帧生效→err=0), mode 字段无协议信息故不用
	if (fb.error_raw == 1u)
	{
		enable_req_ = false;
	}
	if (fb.error_raw == 0u)
	{
		disable_req_ = false;
	}
}

unistackbot_protocol::NodeCommand DmSafety::enable_command() const
{
	// 达妙使能 = 独立特殊帧 (encode→FF×7+FC), 帧无参数——锚定/增益不掺和;
	// 使能后第一拍 MIT 由主站控制流以实测位起锚 (安全怠速先例)
	unistackbot_protocol::NodeCommand cmd;
	cmd.mode = unistackbot_protocol::NodeMode::kEnable;
	return cmd;
}

unistackbot_protocol::NodeCommand DmSafety::disable_command() const
{
	// 失能帧 (encode→FF×7+FD)。警示: 失能=三相释放无保持力, 重力臂下垂——
	// 仅紧急停/运输态; 运行中"停"= 持续 MIT 锚定 (断流语义归框架侧)
	unistackbot_protocol::NodeCommand cmd;
	cmd.mode = unistackbot_protocol::NodeMode::kStop;
	return cmd;
}

unistackbot_protocol::NodeCommand DmSafety::safety_stop_command() const
{
	// **无反馈门** (2026-10-21 二轮审查: last_fb_ 零初始化, kUnknown 期锚定 MIT
	// 将向位置 0 主动上力——驱动器跨主控重启保持使能时是暴力抽动): 无反馈 =
	// 无锚点, 回退失能帧 (失能态驱动器忽略 MIT, 使能态见下); 有反馈后为 A 形态
	// 锚定 + 先验 kp/kd 阻尼 (受控停, 保持力)
	if (state_ == NeutralState::kUnknown)
	{
		return disable_command();
	}
	unistackbot_protocol::NodeCommand cmd;
	cmd.mode = unistackbot_protocol::NodeMode::kRun;
	cmd.position = last_fb_.position;
	cmd.speed = 0.0;
	cmd.kp = unistackbot_protocol::dm_motor::kPriorKp;
	cmd.kd = unistackbot_protocol::dm_motor::kPriorKd;
	return cmd;
}

unistackbot_protocol::NodeCommand DmSafety::idle_command() const
{
	// 安全驻留 (命令流未开始): 同急停形态 —— 维持使能与保持力, 堵
	// "enable 确认 → idle 发失能帧 → 后续 MIT 命令流全被忽略" 链 (2026-10-21 审查 P0);
	// 失能态下此帧被驱动器忽略 (无害)
	return safety_stop_command();
}

}  // namespace unistackbot_statemachine
