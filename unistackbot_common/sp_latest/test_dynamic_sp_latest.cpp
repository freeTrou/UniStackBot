/*
 * test_dynamic_sp_latest —— DynamicSpLatest 组件测试 (2026-10-21, 组合模型 R-a)。
 *
 * 覆盖:
 *   ① 单线程语义: init/publish/read 基本流 + frameKind 序列 (kNone→kInit→kLive)
 *   ② 尺寸契约: publish n≠size 拒 / read cap<size 拒 / null 拒
 *   ③ 逐元素正确性: 多元素写入读出 (含负值/大值), 无交叉错位
 *   ④ 重 init (重装配): 尺寸可变、seq 回 2 (stop/start 场景)
 *   ⑤ 多线程压测: 写者 240Hz 递增自洽帧 (tag+校验和), 读者 480Hz 完整性校验
 *      —— 撕裂 = 帧内不自洽, 计数必须为 0 (有 seqlock 保护则为 0; 去掉宣告即爆,
 *      SpLatest 踩坑记录同款验证法)
 *   ⑥ tears 计数观测
 *
 * 编译运行 (零 ROS, g++ 直编):
 *   cd unistackbot_common/sp_latest
 *   g++ -std=c++17 -O2 -pthread -Wall -Wextra -Wconversion test_dynamic_sp_latest.cpp \
 *       -I.. -o /tmp/test_dsl && /tmp/test_dsl
 * TSAN:
 *   g++ -std=c++17 -O1 -g -fsanitize=thread -pthread test_dynamic_sp_latest.cpp \
 *       -I.. -o /tmp/test_dsl_tsan && TSAN_OPTIONS="halt_on_error=0" /tmp/test_dsl_tsan
 */
#include <atomic>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include "sp_latest/dynamic_sp_latest.hpp"

using unistackbot_common::DynamicSpLatest;

namespace
{

struct Node
{
	std::uint64_t tag = 0;    // 帧号 (写者递增; 读者校验 tag 自洽)
	double q[4] = {};
	std::uint32_t sum = 0;    // 校验和: q[i] 尾数和的量化 (撕裂必不自洽)
};
static_assert(std::is_trivially_copyable_v<Node>);

int g_fail = 0;
int g_case = 0;

void check(bool ok, const char *what)
{
	++g_case;
	if (!ok)
	{
		++g_fail;
		std::printf("FAIL [%d] %s\n", g_case, what);
	}
}

std::uint32_t makeSum(const Node &n)
{
	std::uint32_t acc = 0;
	for (double d : n.q)
	{
		acc = acc * 31u + static_cast<std::uint32_t>(d * 1000.0);
	}
	return acc;
}

}  // namespace

int main()
{
	constexpr std::size_t kN = 24;   // r1_pro 整机形态 (24 电机节点)

	// ---- ① 基本流 + frameKind 序列 ----
	{
		DynamicSpLatest<Node> ch;
		Node out[kN];
		uint64_t seq = 0;
		check(ch.read(out, kN, seq) == false, "未 init read 拒");
		check(ch.seq() == 0 && DynamicSpLatest<Node>::frameKind(ch.seq()) == DynamicSpLatest<Node>::FrameKind::kNone, "seq=0 → kNone");
		ch.init(kN, Node{});
		check(ch.size() == kN, "init 尺寸");
		ch.read(out, kN, seq);
		check(seq == DynamicSpLatest<Node>::kInitFrameSeq, "init 后 seq=2");
		check(DynamicSpLatest<Node>::frameKind(seq) == DynamicSpLatest<Node>::FrameKind::kInit, "init → kInit");
		Node w[kN];
		for (std::size_t k = 0; k < kN; ++k)
		{
			w[k].tag = 7;
			for (int i = 0; i < 4; ++i)
			{
				w[k].q[i] = 0.5 * (i + 1);
			}
			w[k].sum = makeSum(w[k]);
		}
		check(ch.publish(w, kN), "publish ok");
		ch.read(out, kN, seq);
		check(DynamicSpLatest<Node>::frameKind(seq) == DynamicSpLatest<Node>::FrameKind::kLive, "publish → kLive");
		check(out[13].tag == 7 && out[13].sum == makeSum(out[13]), "逐元素读出");
	}

	// ---- ② 尺寸契约 ----
	{
		DynamicSpLatest<Node> ch;
		ch.init(4, Node{});
		Node w[4] = {};
		check(ch.publish(w, 3) == false, "publish n<size 拒");
		check(ch.publish(w, 5) == false, "publish n>size 拒");
		check(ch.publish(nullptr, 4) == false, "publish null 拒");
		Node out[3];
		uint64_t seq = 0;
		check(ch.read(out, 3, seq) == false, "read cap<size 拒");
		check(ch.read(nullptr, 4, seq) == false, "read null 拒");
	}

	// ---- ③ 逐元素 (负值/极值, 24 槽错位探针) ----
	{
		DynamicSpLatest<Node> ch;
		ch.init(kN, Node{});
		Node w[kN];
		for (std::size_t k = 0; k < kN; ++k)
		{
			w[k].tag = 1000 + k;
			for (int i = 0; i < 4; ++i)
			{
				w[k].q[i] = (k % 2 == 0 ? -1.0 : 1.0) * (static_cast<double>(k) + i * 0.25);
			}
			w[k].sum = makeSum(w[k]);
		}
		check(ch.publish(w, kN), "publish 24");
		Node out[kN];
		uint64_t seq = 0;
		check(ch.read(out, kN, seq), "read 24");
		bool all = true;
		for (std::size_t k = 0; k < kN; ++k)
		{
			all = all && out[k].tag == 1000 + k && out[k].sum == makeSum(out[k]);
		}
		check(all, "24 槽逐位自洽 (无错位)");
	}

	// ---- ④ 重 init (重装配) ----
	{
		DynamicSpLatest<Node> ch;
		ch.init(4, Node{});
		Node w[4] = {};
		ch.publish(w, 4);
		uint64_t seq = 0;
		ch.read(w, 4, seq);
		ch.init(8, Node{});   // 重装配: 尺寸可变
		check(ch.size() == 8, "重 init 尺寸 8");
		ch.read(w, 8, seq);
		check(seq == DynamicSpLatest<Node>::kInitFrameSeq, "重 init seq 回 2");
	}

	// ---- ⑤ 多线程压测: 撕裂=帧内不自洽, 必须为 0 ----
	{
		DynamicSpLatest<Node> ch;
		ch.init(kN, Node{});
		std::atomic<bool> stop{false};
		std::atomic<std::uint64_t> writer_frames{0};
		std::atomic<std::uint64_t> reader_ok{0};
		std::atomic<std::uint64_t> reader_stale{0};
		std::thread writer([&] {
			std::uint64_t tag = 0;
			Node w[kN];
			auto next = std::chrono::steady_clock::now();
			const auto period = std::chrono::microseconds(2000);   // 500Hz 贴设计域 (真实写者=总线拍频)
			while (!stop.load())
			{
				++tag;
				for (std::size_t k = 0; k < kN; ++k)
				{
					w[k].tag = tag;
					for (int i = 0; i < 4; ++i)
					{
						w[k].q[i] = static_cast<double>(tag % 100) + i;
					}
					w[k].sum = makeSum(w[k]);
				}
				ch.publish(w, kN);
				writer_frames.fetch_add(1);
				next += period;
				std::this_thread::sleep_until(next);
			}
		});
		Node rd[kN];
		uint64_t seq = 0;
		std::uint64_t torn = 0;
		std::uint64_t checked = 0;
		std::thread reader([&] {
			while (!stop.load())
			{
				if (ch.read(rd, kN, seq))
				{
					for (std::size_t k = 0; k < kN; ++k)
					{
						// 帧内自洽: 24 槽 tag 全等 + 校验和匹配 —— 撕裂必不自洽
						if (rd[k].tag != rd[0].tag || rd[k].sum != makeSum(rd[k]))
						{
							++torn;
							break;
						}
					}
					++checked;
					reader_ok.fetch_add(1);
				}
				else
				{
					reader_stale.fetch_add(1);
				}
			}
		});
		std::this_thread::sleep_for(std::chrono::milliseconds(700));
		stop.store(true);
		writer.join();
		reader.join();
		check(writer_frames.load() > 100 && checked > 100, "压测有量");
		check(torn == 0, "撕裂=0 (seqlock 宣告式保护: 成功帧必自洽)");
		check(ch.tears() <= reader_stale.load(), "tears 口径 ⊆ 降级次数 (失败尝试计数, 非脏帧数)");
		std::printf("  压测: 写 %lu 帧 / 读成功 %lu / 降级 %lu / 撕裂 %lu\n",
			static_cast<unsigned long>(writer_frames.load()),
			static_cast<unsigned long>(reader_ok.load()),
			static_cast<unsigned long>(reader_stale.load()),
			static_cast<unsigned long>(torn));
	}

	// ---- ⑥ readLastFrame 路径 (推荐姿势: 持久缓冲+kNone=上一帧) ----
	{
		DynamicSpLatest<Node> ch;
		ch.init(kN, Node{});
		Node rd[kN];
		uint64_t seq = 0;
		auto kind = ch.readLastFrame(rd, kN, seq);
		check(kind == DynamicSpLatest<Node>::FrameKind::kInit, "LF: init 期 kInit");
		Node w[kN];
		for (std::size_t k = 0; k < kN; ++k)
		{
			w[k].tag = 99;
			w[k].sum = makeSum(w[k]);
		}
		ch.publish(w, kN);
		kind = ch.readLastFrame(rd, kN, seq);
		check(kind == DynamicSpLatest<Node>::FrameKind::kLive && rd[5].tag == 99, "LF: kLive 数据");
	}

	std::printf("%s: %d cases, %d fail\n", g_fail == 0 ? "ALL PASS" : "FAILED", g_case, g_fail);
	return g_fail == 0 ? 0 : 1;
}
