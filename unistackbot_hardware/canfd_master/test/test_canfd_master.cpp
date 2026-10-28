/*
 * CanFdMaster vcan0 E2E 测试 (g++ 直编, 零 ROS; serial test 同型)。
 *
 * 前置: vcan0 已建 (sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0)
 * 模型: 主站 + FakeMotor 同进程双 socket 在 vcan0 对打 (协议往返 + 泵 + 状态机 + 交换全链)。
 *
 * 用例:
 *   [装配] CAN ID 缺失/重复拒绝 / 未知接口拒绝 / 未知协议拒绝
 *   [活性] start 后 tx 每拍必发 (P1 红线) / rx 分发 / state→kReady
 *   [意图] enable→kEnabled / disable→kReady / 命令流位置收敛 (FakeMotor 物理跟踪)
 *   [安全] quick_stop 锚定 (FakeMotor cmd 仍到但受控) / 意图在 FAULT 期失效
 *   [故障] force_err→kFault 粘性 / fault_clear 握手回 READY
 *   [陈旧] silent→stale_cycles 后 kUnknown (信念撤销)
 *   [P8]   FakeMotor 死后 rx 恒 0 而 tx 增长 (无自环污染)
*/

#include <chrono>
#include <cstdio>
#include <thread>

#include "fake_motor.hpp"
#include "unistackbot_canfd/canfd_master.hpp"
#include "unistackbot_bus/master_base.hpp"

using unistackbot_bus::MasterConfig;
using unistackbot_bus::NodeSnapshot;
using unistackbot_bus::NodeSpec;
using unistackbot_protocol::NodeCommand;
using unistackbot_protocol::NodeMode;
using unistackbot_statemachine::NeutralState;
using unistackbot_canfd::CanFdMaster;

namespace
{
int g_case = 0, g_fail = 0;

void check(bool ok, const char *what)
{
	++g_case;
	if (!ok)
	{
		++g_fail;
		std::printf("FAIL [%d] %s\n", g_case, what);
	}
}

void spin(std::int64_t ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

MasterConfig makeCfg(const char *master_if)
{
	MasterConfig cfg;
	cfg.endpoint = master_if;
	cfg.rate_hz = 500.0;
	cfg.protocol = "dm4310";
	cfg.statemachine = "dm4310";
	cfg.thread_priority = 0;   // 测试环境无 rtprio 权限 → rt_tune 优雅降级
	cfg.cpu_affinity = -1;
	cfg.stale_cycles = 50;     // @500Hz = 100ms
	NodeSpec a;
	a.id = 0x01;
	a.ratio = 1.0;
	a.can_send_id = 0x01;
	a.can_recv_id = 0x11;
	NodeSpec b;
	b.id = 0x02;
	b.ratio = 1.0;
	b.can_send_id = 0x02;
	b.can_recv_id = 0x12;
	cfg.nodes = {a, b};
	return cfg;
}

std::vector<NodeSnapshot> states(CanFdMaster &m)
{
	std::vector<NodeSnapshot> out(2);
	(void)m.take_state(out);
	return out;
}
}  // namespace

int main(int argc, char ** argv)
{
	// 接口参数: argv[1]=主站接口 argv[2]=FakeMotor 接口 (默认 can0/can1 物理环回台架;
	// vcan 单接口时传 vcan0 vcan0)
	const char *master_if = argc > 1 ? argv[1] : "can0";
	const char *motor_if = argc > 2 ? argv[2] : "can1";
	std::printf("台架: 主站@%s ↔ FakeMotor@%s (物理环回线)\n", master_if, motor_if);

	// ---- [装配] 校验拒绝路径 (配置错误不碰硬件) ----
	{
		CanFdMaster m;
		MasterConfig cfg = makeCfg(master_if);
		cfg.nodes[0].can_send_id = 0;   // 缺 send_id
		check(!m.start(cfg), "装配: CAN send_id=0 拒绝");
	}
	{
		CanFdMaster m;
		MasterConfig cfg = makeCfg(master_if);
		cfg.nodes[1].can_recv_id = cfg.nodes[0].can_recv_id;   // recv 撞车
		check(!m.start(cfg), "装配: recv_id 重复拒绝 (分发键)");
	}
	{
		CanFdMaster m;
		MasterConfig cfg = makeCfg(master_if);
		cfg.endpoint = "no_such_can";   // 接口不存在
		check(!m.start(cfg), "装配: 未知接口拒绝");
	}
	{
		CanFdMaster m;
		MasterConfig cfg = makeCfg(master_if);
		cfg.protocol = "no_such_proto";
		cfg.statemachine = "dm4310";
		check(!m.start(cfg), "装配: 未知协议拒绝");
	}

	// ---- [活性] 主站 + FakeMotor 对打 ----
	CanFdMaster m;
	MasterConfig cfg = makeCfg(master_if);
	check(m.start(cfg), "start 成功");
	check(m.running(), "running 置位");
	FakeMotor motor1(0x01, 0x11, DmMotorType::kDm4310, motor_if);
	FakeMotor motor2(0x02, 0x12, DmMotorType::kDm4310, motor_if);
	motor1.start();
	motor2.start();
	spin(300);   // ~150 拍
	{
		const auto t = m.telemetry();
		check(t.tx_frames > 100, "P1 红线: 每拍必发 (tx>100 @150 拍)");
		check(t.rx_frames > 100, "rx 分发 (FakeMotor 回显)");
		check(t.rx_rejected == 0 && t.rx_nofit == 0, "零拒收零错投");
		check(t.skipped_cycles == 0, "零追帧跳过");
		auto st = states(m);
		check(st[0].state == NeutralState::kReady, "节点1 kReady (err=0)");
		check(st[1].state == NeutralState::kReady, "节点2 kReady");
	}

	// ---- [意图] enable / 命令流 / disable ----
	m.enable(0x01);
	spin(150);
	check(states(m)[0].state == NeutralState::kEnabled, "enable→kEnabled (err=1)");
	{
		// 命令流: 节点1 MIT 到 q=1.0 (kp=45 kd=2), 节点2 无命令 → idle 帧
		std::vector<NodeCommand> cmds(2);
		cmds[0].mode = NodeMode::kRun;
		cmds[0].position = 1.0;
		cmds[0].speed = 0.0;
		cmds[0].kp = 6.25;
		cmds[0].kd = 5.0;   // 临界阻尼 (ζ=1, kdMax 内)
		cmds[0].tau = 0.0;
		m.publish_cmd(cmds);
		spin(2000);   // ~1000 拍 PD 收敛 (ω=2.5, ~5 时间常数)
		auto st = states(m);
		check(st[0].fb.position > 0.9 && st[0].fb.position < 1.1, "位置收敛 (FakeMotor PD 跟踪)");
		check(st[1].state == NeutralState::kReady, "节点2 idle 帧不使能");
	}
	m.disable(0x01);
	spin(150);
	check(states(m)[0].state == NeutralState::kReady, "disable→kReady");

	// ---- [安全] quick_stop ----
	m.enable(0x01);
	spin(150);
	m.quick_stop();
	spin(150);
	{
		auto st = states(m);
		check(st[0].fb.position < 1.2, "急停后位置受控 (锚定不砸)");
		const auto t = m.telemetry();
		check(t.tx_frames > 400, "急停期仍发帧 (safety_stop 形态, P1)");
	}
	// stop/start 重启复位急停闩锁
	m.stop();
	check(!m.running(), "stop 后 running 清零");
	check(m.start(cfg), "重启 start 成功");
	spin(200);
	check(states(m)[0].state == NeutralState::kReady, "重启后 FakeMotor 回显恢复");

	// ---- [故障] force_err → FAULT 粘性 → fault_clear 握手 ----
	motor1.set_force_err(0x8);   // 过压
	spin(200);
	check(states(m)[0].state == NeutralState::kFault, "err=8 → kFault");
	motor1.set_force_err(0);   // 电机侧恢复干净
	spin(200);
	check(states(m)[0].state == NeutralState::kFault, "FAULT 粘性 (不自动恢复)");
	// FAULT 期 enable 意图失效 (P0: 维护后不惊喜上电)
	m.enable(0x01);
	spin(150);
	motor1.set_force_err(0);
	m.fault_clear();
	spin(250);
	check(states(m)[0].state == NeutralState::kReady, "fault_clear 握手 → kReady");

	// ---- [陈旧] silent → kUnknown 信念撤销 ----
	motor1.set_silent(true);
	spin(300);   // > stale_cycles (100ms)
	check(states(m)[0].state == NeutralState::kUnknown, "silent→kUnknown (陈旧覆盖)");
	check(m.state() == NeutralState::kUnknown, "段级 state worst-of → kUnknown");
	motor1.set_silent(false);
	spin(200);
	check(states(m)[0].state == NeutralState::kReady, "恢复回显 → kReady");

	// ---- [P8] 无自环污染: 主站独跑 (FakeMotor 双双 silent) ----
	{
		motor1.set_silent(true);
		motor2.set_silent(true);
		spin(100);   // 静默前的在途帧落账 (竞态防御: rx0 快照前吃掉 straggler)
		const std::uint64_t rx0 = m.telemetry().rx_frames;
		const std::uint64_t tx0 = m.telemetry().tx_frames;
		spin(200);
		const auto t = m.telemetry();
		check(t.rx_frames == rx0, "P8: 无自环 (rx 恒 0 增量)");
		check(t.tx_frames > tx0 + 50, "P8: tx 持续增长 (发帧不依赖回显)");
		motor1.set_silent(false);
		motor2.set_silent(false);
	}

	m.stop();
	std::printf("%s: %d cases, %d fail\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_case, g_fail);
	return g_fail == 0 ? 0 : 1;
}
