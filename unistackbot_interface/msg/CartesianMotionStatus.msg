# CartesianMotionController 状态反馈 (位姿空间闭环量 + 求解器健康)
#
# 反馈三路各归其位 (2026-09-17 定稿): 原始关节弧度一律看 /joint_states
# (joint_state_broadcaster, 关节空间单一事实源); 本消息只发位姿空间与
# 求解器语义; 控制器不发"命令关节值" (与 /joint_states 形成双源污染)。
# 语义: base 系; 位姿控制是开环发目标闭环看反馈 —— 误差/收敛/失败码是
# 消费者的必需品而非可选项。
# 发布: 控制器 inactive 时发一次 INACTIVE (消费者可辨别失联 vs 停控)。

std_msgs/Header header

uint8 INACTIVE = 0	# 控制器未激活
uint8 IDLE	= 1	# 激活, 无目标
uint8 TRACKING	= 2	# 跟踪目标中
uint8 HOLD	= 3	# 冻结 (来自 ~/pause; 门控语义 2026-10-12 裁决: 停推进但
                #   照常吸收新目标, 解除后追最新 —— target_pose 在 HOLD
                #   期间会更新, 消费者勿依赖其冻结)
uint8 DEGRADED	= 4	# 连续求解失败, 保持上一命令 (可观测降级)

uint8 mode
bool converged		# 容差 = 控制器参数 converge_pos_tol/rot_tol (默认
                    #   1mm / 0.01rad, per-robot yaml 可调); 物理链 (mujoco)
                    #   稳态贴容差线时可能闪烁抖动
float64 position_error	# [m] 当前 FK 位姿与目标的位置误差
float64 rotation_error	# [rad] 姿态误差 (符号无关标量, 内部已 abs)
float64 min_sigma	# 雅可比最小奇异值; **-1 = 求解器不提供** (trac_ik 路径
                    #   恒 -1, 仅 dls_ik 遥测有效 —— 数值解机型默认 trac_ik,
                    #   勿当恒有效仪表: 按其值分支的告警逻辑会失效)
bool timed_out		# 最近一次求解是否预算耗尽 (Distance 档烧满预算的伴随
                    #   标志; 与 last_result 联合分流拒绝原因)
uint8 last_result	# IkResult 枚举值 (unistackbot_interface/ik_result.hpp)

geometry_msgs/Pose current_pose	# FK(当前关节), base 系
geometry_msgs/Pose target_pose	# 当前采纳目标 (无目标 = 等于 current_pose)
# 四元数符号警示 (两 pose 字段共用): 符号未规范化 —— q 与 -q 等价, 发布侧
# 不保证取哪个。消费者自算姿态差/插值前先 abs(dot) 或符号对齐, 否则同一
# 姿态可被算成 ~168° 差; 只用 rotation_error 标量则无碍。
