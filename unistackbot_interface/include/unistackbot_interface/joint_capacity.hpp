#ifndef UNISTACKBOT_INTERFACE__JOINT_CAPACITY_HPP_
#define UNISTACKBOT_INTERFACE__JOINT_CAPACITY_HPP_

#include <cstdint>

namespace unistackbot_interface
{

// 关节容量上限: 反馈帧/指令帧/仿真命令帧的定长数组预留空间, 唯一定义。
//
// 为什么是 32: r1_pro 轮式双臂 18 命令关节实证扩容 (2026-10-14, 16 撞界) ——
// 覆盖双臂+腰+手部余量; 单臂机型 (piper 9/xarm7 7) 远在内。
//
// 为什么不是"恰好 N": C++ 类型尺寸是编译期事实, 配置文件(URDF/yaml)配置的是
// 运行时数量 —— joint_count 字段与插件内部 vector 的恰好 resize; 类型布局无法
// 从配置读。此常量是**协议格式规格**(同 CAN FD 帧的 data[64] 之于 DLC), 不是
// 机器人参数; 业务参数住配置, 协议规格住代码。
//
// 升容量的路径 (如将来上 43-DoF 人形): 改此数 + 全仓重编 + verify —— monorepo
// 无外部消费者、帧不跨进程不序列化, 变更成本 = 一次构建 (2026-09-17 裁决)。
// 帧大小 @32: RobotCommand ≈ 1.6 KB / RobotFeedback ≈ 0.9 KB。
inline constexpr uint32_t kMaxJoints = 32;

}  // namespace unistackbot_interface
#endif  // UNISTACKBOT_INTERFACE__JOINT_CAPACITY_HPP_
