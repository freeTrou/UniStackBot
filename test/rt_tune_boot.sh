#!/usr/bin/env bash
# RT 核开机调优 — 禁深睡 + performance governor + uncore 锁频。
# x86 通用: idle 档按"退出延迟值"取舍 (兼容 ACPI 4 档表 / intel_idle 多档表差异), 不依赖档位索引与命名。
# 用法:  sudo bash test/rt_tune_boot.sh [0-3]
#   不传参数 → DEFAULT_CPUS (当前 = 核0,1,2,3 前四核全调优; 换机器改下面 DEFAULT_CPUS)
#   传 N (0-3) → 核 0..N (显式指定范围, 覆盖默认)
# idle 策略: 退出延迟 > MAX_EXIT_US 的档全禁 (本机=禁 C2 127µs/C3 1048µs, 留 C1 1µs)。
# 恢复 (示例核1,2):
#   idle     for s in /sys/devices/system/cpu/cpu{1,2}/cpuidle/state*/; do echo 0 > "$s/disable"; done
#   governor echo powersave | sudo tee /sys/devices/system/cpu/cpu{1,2}/cpufreq/scaling_governor
#   uncore   echo <initial_min_freq_khz 的值> | sudo tee .../intel_uncore_frequency/package_00_die_00/min_freq_khz
# (生产环境时改为 systemd oneshot service; 依据与实测见 test/results/rt_baseline_isolcpus.md)
# 直接写 sysfs, 零依赖 — cpupower 是内核版本配套包 (linux-tools-<ver>), 换内核即断供。
set -euo pipefail

DEFAULT_CPUS="0 1 2 3"   # 默认调优核 (2026-09-20 用户裁定: 前四核; 换机器改这里)
MAX_EXIT_US=2        # 禁用退出延迟大于此值 (µs) 的 idle 档
SCOPE="${1:-}"

if [ -n "$SCOPE" ]; then
  case "$SCOPE" in
    0|1|2|3) CPUS=$(seq 0 "$SCOPE") ;;
    *) echo "参数须为 0-3 (代表核 0..N), 收到: $SCOPE"; exit 1 ;;
  esac
else
  CPUS="$DEFAULT_CPUS"
fi
FIRST_CPU="${CPUS%% *}"

if [ "$(id -u)" -ne 0 ]; then
  echo "需要 root: sudo bash $0 [0-3]"
  exit 1
fi
[ -d "/sys/devices/system/cpu/cpu$FIRST_CPU/cpuidle" ] || { echo "本机无 cpuidle sysfs (虚拟机/BIOS 禁 idle? ), 不适用"; exit 1; }

# ---------- ① idle: 按退出延迟禁深档 (留浅档如 C1/POLL) ----------
for c in $CPUS; do
  for s in /sys/devices/system/cpu/cpu$c/cpuidle/state*/; do
    if [ "$(cat "$s/latency")" -gt "$MAX_EXIT_US" ]; then
      echo 1 > "$s/disable"
    else
      echo 0 > "$s/disable"
    fi
  done
done
echo "✓ 核[$CPUS] 已禁退出延迟>${MAX_EXIT_US}µs 的 idle 档 — 空闲只留浅档"

# ---------- ② 性能模式 (仅所选核; 其余核保持 powersave 随负载调频) ----------
for c in $CPUS; do
  AVAIL="/sys/devices/system/cpu/cpu$c/cpufreq/scaling_available_governors"
  if ! grep -qw performance "$AVAIL" 2>/dev/null; then
    echo "⚠️ cpu$c 无 performance governor (可用: $(cat "$AVAIL" 2>/dev/null || echo '?')), 跳过"
    continue
  fi
  echo performance > "/sys/devices/system/cpu/cpu$c/cpufreq/scaling_governor"
done
echo "✓ 核[$CPUS] governor=performance"

# ---------- ③ EPP (Intel HWP 机器才有此文件, 无则跳过) ----------
for c in $CPUS; do
  EPP="/sys/devices/system/cpu/cpu$c/cpufreq/energy_performance_preference"
  if [ -f "$EPP" ] && [ "$(cat "$EPP")" != "performance" ]; then
    echo performance > "$EPP"
  fi
done

# ---------- ④ uncore 锁频 (默认关: 2026-09-20 三档对照实测无可辨识收益, 见基线 D 节; 复测改 LOCK_UNCORE=1) ----------
LOCK_UNCORE=0
UNCORE_DIR=$(ls -d /sys/devices/system/cpu/intel_uncore_frequency/package_*_die_* 2>/dev/null | head -1)
if [ "$LOCK_UNCORE" = "1" ] && [ -n "$UNCORE_DIR" ]; then
  UMAX=$(cat "$UNCORE_DIR/initial_max_freq_khz")
  echo "$UMAX" > "$UNCORE_DIR/min_freq_khz"
  echo "✓ uncore min=$UMAX kHz (=max 锁频)"
elif [ -n "$UNCORE_DIR" ]; then
  echo "ℹ uncore 锁频默认关 (对照无收益); 当前 min=$(cat "$UNCORE_DIR/min_freq_khz") kHz"
else
  echo "ℹ 无 uncore sysfs 接口 (非 Intel 或旧内核), 跳过"
fi

# ---------- 验证 (档名[退出延迟]后 0=可用 1=已禁) ----------
echo "----- 验证 -----"
for c in $CPUS; do
  G=$(cat "/sys/devices/system/cpu/cpu$c/cpufreq/scaling_governor" 2>/dev/null || echo 无)
  IDLE=$(for s in /sys/devices/system/cpu/cpu$c/cpuidle/state*/; do
    printf '%s[%sµs](%s) ' "$(cat "$s/name")" "$(cat "$s/latency")" "$(cat "$s/disable")"
  done)
  echo "cpu$c: governor=$G  idle: $IDLE"
done
if [ -n "$UNCORE_DIR" ]; then
  echo "uncore: min=$(cat "$UNCORE_DIR/min_freq_khz") max=$(cat "$UNCORE_DIR/max_freq_khz") kHz"
fi
