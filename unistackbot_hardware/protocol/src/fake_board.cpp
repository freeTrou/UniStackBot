#include "unistackbot_protocol/fake_board.hpp"

namespace unistackbot_protocol
{

ResponseFrameBits FakeBoard::process(const ControlFrameBits & cmd)
{
	ResponseFrameBits rsp{};
	rsp.ts = cmd.ts;   // ts 回显 (HAL 侧按序号对账)

	// 臂/头 (bus[0..2]): 命令回声 —— tar→cur, 模式/使能/心跳如实回
	for (size_t b = 0; b < kArmBusCount; ++b)
	{
		rsp.bus[b] = {};
		for (size_t m = 0; m < kArmMotorsPerBus; ++m)
		{
			const MotorCmdBits & c = cmd.bus[b].motor[m];
			MotorStateBits & s = rsp.bus[b].motor[m];
			s.status = cmd.bus[b].ctrl_word;
			s.enable = c.enable;
			s.heart_beat = 1;
			s.error = 0;
			s.temp = 25;
			s.cur_tor = c.tar_tor;
			s.cur_vel = c.tar_vel;
			s.cur_pos = c.tar_pos;
		}
	}

	// 底盘: 实录 golden 假数据规则 (与 fake_motor.py 逐字段一致; Response 底盘段无 ctrl_word)
	rsp.bus_chassis = {};
	for (size_t i = 0; i < kChassisMotorCount; ++i)
	{
		MotorStateChassisBits & s = rsp.bus_chassis.motor[i];
		s.cur_vel = static_cast<int32_t>(1000 + i);
		s.cur_pos = static_cast<int32_t>(5000 + static_cast<int>(i) * 100);
		s.cur_tor = static_cast<int16_t>(i);
		s.status = kCtrlWordPdMit;
		s.error = 0;
	}

	++frames_;
	return rsp;
}

}  // namespace unistackbot_protocol
