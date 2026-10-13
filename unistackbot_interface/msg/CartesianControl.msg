# CartesianMotionController 控制通道 (事件语义: 行为状态变更才发布, TRANSIENT_LOCAL)
#
# 通道分工 (2026-09-17 定稿):
#   ~/command  geometry_msgs/PoseStamped  值通道  高频, 最新值即全部语义 (base 系)
#   ~/pause 本消息                     事件通道 低频, 变更沿才有信息
# 判据法则: 跟位姿一起变的字段进 target 侧; 行为状态变更进本消息 (v2 每目标
# 速度限制属于前者)。未收到本消息 = 默认 TRACKING。
# RT 消费: mode 以原子标志进控制环; HOLD=运动推进门控 (2026-10-12 裁决):
# 冻结推进 (OTG 自目标 C2 停), 照常接收/求解/采纳新目标 (goal_ 更新),
# 解除后从当前状态重规划 C2 追最新目标。
# 跨 topic 无顺序保证: HOLD 与最后一条 command 最坏有 1 周期歧义, 位移由
# 控制器 max_cart_step 饱和封顶 (有界且无害)。

uint8 TRACKING = 0	# 正常跟踪 ~/command
uint8 HOLD	= 1	# 冻结推进 (软停止, 不切控制器; 门控语义见上)

uint8 mode
