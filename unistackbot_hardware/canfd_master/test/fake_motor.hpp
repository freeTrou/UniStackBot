#ifndef UNISTACKBOT_CANFD__TEST_FAKE_MOTOR_HPP_
#define UNISTACKBOT_CANFD__TEST_FAKE_MOTOR_HPP_

/*
 FakeMotor —— vcan 上的虚拟达妙电机 (测试侧, 不进库; 设计 §8.6)。
 单 socket 线程: 订阅 send_id → unpackCommand (协议镜像函数) → 一阶物理 →
 packFeedback 回写 recv_id。故障注入旋钮: silent (不应答 → 主站陈旧) /
 force_err (强制错误码 → FAULT) / delay_us (慢响应)。
 与主站共用同一份 dm_motor_core —— 协议往返一致性由构造保证, 对真机保真
 留给第一台真机 (镜像函数测试的头文件注释同款诚实边界)。
*/

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <thread>

#include "unistackbot_protocol/dm_motor_core.hpp"

using unistackbot_protocol::NodeCommand;
using unistackbot_protocol::NodeFeedback;
using unistackbot_protocol::NodeMode;
using unistackbot_protocol::dm_motor::DmMotorType;
using unistackbot_protocol::dm_motor::packFeedback;
using unistackbot_protocol::dm_motor::unpackCommand;

class FakeMotor
{
public:
	FakeMotor(std::uint32_t send_id, std::uint32_t recv_id, DmMotorType type, const char *ifname)
		: send_id_(send_id), recv_id_(recv_id), type_(type)
	{
		sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		ioctl(sock_, FIONBIO, &nonblock_);   // 非阻塞轮询 (测试线程, 简单优先)
		sockaddr_can addr{};
		addr.can_family = AF_CAN;
		addr.can_ifindex = static_cast<int>(if_nametoindex(ifname));
		bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
		can_filter flt[1] = {{send_id_, 0x7FFu}};
		setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_FILTER, flt, sizeof(flt));
	}

	~FakeMotor()
	{
		stop_ = true;
		if (thread_.joinable())
		{
			thread_.join();
		}
		if (sock_ >= 0)
		{
			::close(sock_);
		}
	}

	void start()
	{
		thread_ = std::thread([this] { loop(); });
	}

	// ── 故障注入旋钮 ──
	void set_silent(bool on) { silent_ = on; }
	void set_force_err(std::uint32_t err) { force_err_ = err; }   // 0 = 不覆盖
	void set_delay_us(int us) { delay_us_ = us; }

	// 观测 (测试断言用)
	[[nodiscard]] double q() const { return q_; }
	[[nodiscard]] std::uint64_t cmd_count() const { return cmd_count_; }

private:
	void loop()
	{
		while (!stop_)
		{
			can_frame cf{};
			const ssize_t r = ::read(sock_, &cf, sizeof(cf));
			if (r != static_cast<ssize_t>(sizeof(cf)))
			{
				std::this_thread::sleep_for(std::chrono::microseconds(200));
				continue;
			}
			++cmd_count_;
			NodeCommand cmd;
			if (!unpackCommand(cf.data, cf.can_dlc, cmd, 1.0, type_))
			{
				continue;   // 非法帧不应答 (主站计 rx_rejected 无——收不到就没有)
			}
			// 使能/失能/清故障状态推进 (真达妙语义: 0xFB 清故障后回失能态 err=0)
			if (cmd.mode == NodeMode::kEnable)
			{
				enabled_ = true;
			}
			else if (cmd.mode == NodeMode::kStop || cmd.mode == NodeMode::kClear)
			{
				enabled_ = false;
			}
			// 物理推进: MIT PD 一阶 (τ = τ_ff + kp(q_des−q) + kd(dq_des−dq); 惯量 1.0)
			if (cmd.mode == NodeMode::kRun && enabled_)
			{
				const double q = q_.load();
				const double dq = dq_.load();
				const double tau = cmd.tau + cmd.kp * (cmd.position - q) + cmd.kd * (cmd.speed - dq);
				dq_.store(dq + tau * kDt);
				q_.store(q + dq_.load() * kDt);
			}
			if (delay_us_ > 0)
			{
				std::this_thread::sleep_for(std::chrono::microseconds(delay_us_));
			}
			if (silent_)
			{
				continue;   // 不应答 → 主站看门狗路径
			}
			NodeFeedback fb;
			fb.node_id = 0;   // 主站按 CAN ID 分发, D0 低半字节不参与路由
			fb.error_raw = force_err_.load() != 0 ? force_err_.load() : (enabled_ ? 1u : 0u);
			fb.position = q_.load();
			fb.speed = dq_.load();
			fb.tau = 0.0;
			fb.temp_driver = 30;
			fb.temp_winding = 31;
			std::uint8_t buf[8];
			if (packFeedback(buf, sizeof(buf), fb, 1.0, type_) == 8)
			{
				can_frame out{};
				out.can_id = recv_id_;
				out.can_dlc = 8;
				std::memcpy(out.data, buf, 8);
				::write(sock_, &out, sizeof(out));
			}
		}
	}

	static constexpr double kDt = 0.002;   // 物理步长 = 命令到达率倒数 (500Hz)
	std::uint32_t send_id_, recv_id_;
	DmMotorType type_;
	int sock_ = -1;
	int nonblock_ = 1;
	std::thread thread_;
	std::atomic<bool> stop_{false};
	std::atomic<bool> silent_{false};
	std::atomic<std::uint32_t> force_err_{0};
	std::atomic<int> delay_us_{0};
	std::atomic<bool> enabled_{false};
	std::atomic<double> q_{0.0};
	std::atomic<double> dq_{0.0};
	std::atomic<std::uint64_t> cmd_count_{0};
};

#endif
