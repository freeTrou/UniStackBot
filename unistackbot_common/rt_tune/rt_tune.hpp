#ifndef UNISTACKBOT_COMMON__RT_TUNE_HPP_
#define UNISTACKBOT_COMMON__RT_TUNE_HPP_

#include <unistd.h>
#include <sched.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <pthread.h>

namespace unistackbot_common
{

/*
 * RtTune —— RT 线程调优的参数化应用 (2026-09-20, 阶段0 配套)。
 *
 * 供控制器在**自己的 RT 线程内**调用 (update() 首拍或线程入口)——
 * 亲和性与调度策略只能作用于调用线程自身 (pthread_self), 不能跨线程设置。
 *
 * 参数语义:
 *   cpu        -1 = 不绑定 (跟随调度器); 0..N-1 = 绑定指定核
 *   fifo_prio   0 = 不动调度策略 (SCHED_OTHER);
 *              >0 = SCHED_FIFO 该优先级 (需 root/CAP_SYS_NICE, 失败降级 SCHED_OTHER 并 WARN)
 *   nice_val    SCHED_OTHER 下的 nice 值 (仅 fifo_prio=0 时应用; FIFO 忽略 nice)
 *   name        线程名 (prctl, top/gdb 可见; nullptr = 不设)
 *
 * 返回值: 0 = 全部成功; 否则按位或的错误码 (见 enum)。失败不抛不崩 ——
 * 调优是性能优化, 不是正确性前提。
 */
namespace rt_tune
{

enum Errors : int
{
	kOk = 0,
	kAffinityFailed = 1,    // 绑核失败 (核号越界/权限)
	kFifoFailed = 2,        // SCHED_FIFO 失败 (EPERM 为主, 降级 SCHED_OTHER)
	kNiceFailed = 4,        // nice 设置失败 (非特权加不高, 只能加低)
};

inline int apply(int cpu, int fifo_prio, int nice_val, const char * name)
{
	int rc = rt_tune::kOk;
	const long tid = ::syscall(SYS_gettid);

	if (name != nullptr)
	{
		prctl(PR_SET_NAME, name, 0, 0, 0);
	}

	// ---- 绑核 ----
	if (cpu >= 0)
	{
		cpu_set_t set;
		CPU_ZERO(&set);
		CPU_SET(cpu, &set);
		const int aff_rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
		if (aff_rc != 0)
		{
			rc |= rt_tune::kAffinityFailed;
			std::fprintf(stderr, "rt_tune[%s]: 绑核 %d 失败 (%s) — 继续未绑定\n",
				name ? name : "?", cpu, std::strerror(aff_rc));
		}
	}

	// ---- 调度策略 ----
	if (fifo_prio > 0)
	{
		sched_param sp{};
		sp.sched_priority = fifo_prio;
		if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
		{
			rc |= rt_tune::kFifoFailed;
			// 降级: SCHED_OTHER + 可配 nice (开发机无 root 的常态路径)
			std::fprintf(stderr,
				"rt_tune[%s]: SCHED_FIFO(%d) 失败 (%s) — 降级 SCHED_OTHER (生产需 root/cap_sys_nice)\n",
				name ? name : "?", fifo_prio, std::strerror(errno));
			if (nice_val != 0)
			{
				if (setpriority(PRIO_PROCESS, static_cast<id_t>(tid), nice_val) != 0)
				{
					rc |= rt_tune::kNiceFailed;
				}
			}
		}
		else if (nice_val != 0)
		{
			// FIFO 成功时 nice 无意义但设置无害; 保持语义干净: 忽略
		}
	}
	else if (nice_val != 0)
	{
		if (setpriority(PRIO_PROCESS, static_cast<id_t>(tid), nice_val) != 0)
		{
			rc |= rt_tune::kNiceFailed;
		}
	}

	// ---- 应用结果回显 (一次性, 验证面: top -H / taskset -pc) ----
	int policy = sched_getscheduler(0);
	sched_param cur{};
	sched_getparam(0, &cur);
	cpu_set_t got;
	pthread_getaffinity_np(pthread_self(), sizeof(got), &got);
	char cores[256];
	cores[0] = '\0';
	for (int c = 0; c < CPU_SETSIZE; ++c)
	{
		if (CPU_ISSET(c, &got))
		{
			char one[8];
			std::snprintf(one, sizeof(one), "%s%d", cores[0] ? "," : "", c);
			std::strcat(cores, one);
		}
	}
	std::fprintf(stderr, "rt_tune[%s]: tid=%ld policy=%s prio=%d cores=[%s]\n",
		name ? name : "?", tid,
		(policy == SCHED_FIFO) ? "FIFO" : (policy == SCHED_RR ? "RR" : "OTHER"),
		cur.sched_priority, cores);
	return rc;
}

}  // namespace rt_tune
}  // namespace unistackbot_common
#endif  // UNISTACKBOT_COMMON__RT_TUNE_HPP_
