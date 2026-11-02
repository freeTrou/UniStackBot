#ifndef UNISTACKBOT_UDP_BOARD__FAKE_BOARD_HPP_
#define UNISTACKBOT_UDP_BOARD__FAKE_BOARD_HPP_
/*
 * FakeBoard —— 虚拟电机板 (Phase 0B 仿真侧件, 零硬件)
 *
 * 对等移植自 mos_hal2.0 debug/fake_motor.py (@single-motor-control 分支), 两处增强:
 *   ① 臂/头电机按"命令回声"响应 (tar→cur, 一阶跟随零延迟) —— python 版只填底盘,
 *      臂侧回声让臂链路端到端可测 (Phase 3 消费);
 *   ② 纯函数核心 (process) + UDP 壳分离 —— 编解码零 IO 可测, RTT 自测走壳。
 *
 * 底盘假数据规则 = 实录 golden (example_run_frames.md §2):
 *   电机 i (0..9): cur_vel=1000+i, cur_pos=5000+i*100, cur_tor=i, status=3(PD/MIT), error=0
 */
#include <cstdint>
#include <stddef.h>

#include "board_protocol.hpp"

namespace unistackbot_protocol
{

class FakeBoard
{
public:
	// 命令帧 → 反馈帧 (纯函数核心: 假数据规则 + ts 回显 + 臂回声; 调用方保证帧已解码合法)
	[[nodiscard]] ResponseFrameBits process(const ControlFrameBits & cmd);

	// 处理帧计数 (观测用)
	[[nodiscard]] uint64_t framesHandled() const { return frames_; }

private:
	uint64_t frames_{0};
};

}  // namespace unistackbot_protocol
#endif  // UNISTACKBOT_UDP_BOARD__FAKE_BOARD_HPP_
