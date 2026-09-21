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
uint8 HOLD	= 3	# 冻结 (来自 ~/control)
uint8 DEGRADED	= 4	# 连续求解失败, 保持上一命令 (可观测降级)

uint8 mode
bool converged
bool stream_stale	# ~/target 断流 (StaleWatch 判定; 断流受控减速中/已停, 0c)
float64 position_error	# [m] 当前 FK 位姿与目标的位置误差
float64 rotation_error	# [rad] 姿态误差
float64 min_sigma	# 雅可比最小奇异值 (奇异接近度透传, dls_ik 遥测)
bool timed_out		# 最近一次求解是否预算耗尽 (分流: NEAR_SINGULAR x timed_out)
uint8 last_result	# IkResult 枚举值 (unistackbot_interface/ik_result.hpp)

geometry_msgs/Pose current_pose	# FK(当前关节), base 系
geometry_msgs/Pose target_pose	# 当前采纳目标 (无目标 = 等于 current_pose)
