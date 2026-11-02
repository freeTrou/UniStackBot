/*
 * BaseKinematics 实现的宿主 (2026-11-01 从 Pkg 1 编译期迁入 Pkg 2)
 * diff + n_steer 的 .cpp 编入本 .so; 工厂注册经静态初始化自动完成
 * (registerBaseKinematics 在 base_kinematics.hpp 声明, 实现/注册在此)。
 * BaseVelocityController 调 createBaseKinematics("n_steer") 运行期发现。
 */
#include "base_kinematics/base_kinematics.hpp"

// 注册入口在 .cpp 内（匿名 namespace 类 + lambda 注册, 不需要 pluginlib EXPORT）
// diff 和 n_steer 经 base_kinematics.cpp + n_steer_kinematics.cpp 的静态初始化注册
