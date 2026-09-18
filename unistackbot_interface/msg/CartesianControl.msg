# CartesianMotionController 控制通道 (事件语义: 行为状态变更才发布, TRANSIENT_LOCAL)
#
# 通道分工 (2026-09-17 定稿):
#   ~/target  geometry_msgs/PoseStamped  值通道  高频, 最新值即全部语义 (base 系)
#   ~/control 本消息                     事件通道 低频, 变更沿才有信息
# 判据法则: 跟位姿一起变的字段进 target 侧; 行为状态变更进本消息 (v2 每目标
# 速度限制属于前者)。未收到本消息 = 默认 TRACKING。
# RT 消费: mode 以原子标志进控制环; HOLD 冻结当前命令并忽略新 target。
# 跨 topic 无顺序保证: HOLD 与最后一条 target 最坏有 1 周期歧义, 位移由
# 控制器 max_cart_step 饱和封顶 (有界且无害)。

uint8 TRACKING = 0	# 正常跟踪 ~/target
uint8 HOLD	= 1	# 冻结当前命令, 忽略新 target (软停, 不切控制器)

uint8 mode
