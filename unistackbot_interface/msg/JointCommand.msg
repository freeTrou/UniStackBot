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
float64[] effort                  # [N·m] 力矩通道, 角色随 mode: CST=主载荷 / CSP·MIT=前馈 τ_ff。
                                  # 与 JointState.effort (测量值) 同词不同义: 此处=要发的。
                                  # 前馈消费随真机 MIT 批次启用 (仿真链 position 接口表达不了力矩)。
float64[] kp                      # MIT 比例增益 [N·m/rad] (0=不启用); 关节侧 (输出端) 单位,
                                  # 转子侧换算 (×ratio²) 归协议层
float64[] kd                      # MIT 阻尼增益 [N·m·s/rad]; 单位口径同 kp
