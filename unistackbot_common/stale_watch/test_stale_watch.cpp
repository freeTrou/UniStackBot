// StaleWatch 功能测试 (0c 断流看门狗)。
// 独立编译, 零依赖:
//   g++ -std=c++17 -O2 -Wall -Wextra -Wconversion test_stale_watch.cpp -o test_stale_watch && ./test_stale_watch
// 核心断言: ① 无流=IDLE 不是 STALE ② 阈值拍后 STALE ③ seq 恢复即 LIVE
//           ④ 阈值 0 = 永不 STALE ⑤ 重复值消息也算活着 ⑥ reset 回 IDLE

#include "stale_watch.hpp"

#include <cstdio>

namespace
{

using unistackbot_common::StaleWatch;

int g_failures = 0;

void check(bool ok, const char * name)
{
	std::printf("%s: %s\n", ok ? "PASS" : "FAIL", name);
	if (!ok)
	{
		++g_failures;
	}
}

}  // namespace

int main()
{
	// ① 无流 = IDLE (待命不是故障)
	StaleWatch w1(3);
	check(w1.tick(0) == StaleWatch::State::IDLE, "no-stream: IDLE not STALE");
	check(w1.tick(0) == StaleWatch::State::IDLE, "no-stream: stays IDLE");

	// ② 首条消息 → LIVE; 停发阈值拍后 → STALE (阈值=3: 第 4 拍不变才 STALE, > 判定)
	check(w1.tick(4) == StaleWatch::State::LIVE, "first msg: LIVE");
	check(w1.tick(4) == StaleWatch::State::LIVE, "repeat value: still LIVE (消息活着≠值在变)");
	check(w1.tick(4) == StaleWatch::State::LIVE, "within threshold: LIVE");
	check(w1.tick(4) == StaleWatch::State::LIVE, "threshold boundary (3rd quiet): LIVE");
	check(w1.tick(4) == StaleWatch::State::STALE, "past threshold: STALE");
	check(w1.tick(4) == StaleWatch::State::STALE, "stays STALE");
	check(w1.cyclesSinceUpdate() == 5, "cyclesSinceUpdate counts quiet cycles");

	// ③ seq 恢复 → 立即 LIVE
	check(w1.tick(6) == StaleWatch::State::LIVE, "resume: LIVE on next msg");

	// ④ 阈值 0 = 功能关闭 (永不 STALE)
	StaleWatch w0(0);
	w0.tick(1);
	for (int i = 0; i < 100; ++i)
	{
		w0.tick(1);
	}
	check(w0.state() == StaleWatch::State::LIVE, "disabled (cycles=0): never STALE");

	// ⑤ 高频流不会误判: 每 2 拍一条消息, 阈值 3 → 永远 LIVE
	StaleWatch w2(3);
	w2.tick(2);
	uint64_t seq = 2;
	bool always_live = true;
	for (int i = 0; i < 1000; ++i)
	{
		if ((i % 2) == 0)
		{
			++seq;
		}
		if (w2.tick(seq) != StaleWatch::State::LIVE)
		{
			always_live = false;
		}
	}
	check(always_live, "half-rate stream: LIVE forever (阈值容许 3 拍静默)");

	// ⑥ reset 回 IDLE, 忘掉旧流
	w1.reset();
	check(w1.state() == StaleWatch::State::IDLE, "reset: back to IDLE");
	check(w1.tick(9) == StaleWatch::State::LIVE, "reset: new seq re-LIVE");

	std::printf("%s\n", g_failures == 0 ? "RESULT: ALL PASS" : "RESULT: FAILED");
	return g_failures == 0 ? 0 : 1;
}
