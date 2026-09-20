# 关节级流式命令 (2026-09-18 定稿): 取代 JTC action 的 topic 通道。
# 语义 = 点流 (每条消息是"最新目标", 控制器保持/插值到下一条)。
# QoS 契约: reliable + KeepLast(1) —— 可靠到达且只留最新。
# 与 C++ 契约 RobotCommand (robot_command.hpp) 字段对齐 —— 单一契约两载体。
# 模式语义 (CiA402 对象):
#   CSP: position 主载荷; velocity/effort 可作前馈
#   CSV: velocity 主载荷; effort 可作前馈
#   CST: effort 主载荷
#   MIT: tau = kp*(q_des-q) + kd*(qdot_des-qdot) + tau_ff
# 仿真链 (position 命令接口) 仅消费 CSP; CSV/CST/MIT 需真机硬件接口。

uint8 MODE_NONE=0
uint8 MODE_CSP=1
uint8 MODE_CSV=2
uint8 MODE_CST=3
uint8 MODE_MIT=4

uint8 mode
string[] joint_names              # 关节名序 (控制器按名映射到命令接口)
float64[] position                # [rad]
float64[] velocity                # [rad/s]
float64[] effort                  # [N·m] (tff)
float64[] kp                      # MIT 比例增益 [N·m/rad] (0=不启用)
float64[] kd                      # MIT 阻尼增益 [N·m·s/rad]
