# 统一错误状态板 (单一消息, 三错误域分区块; P1 错误契约出口, 2026-10-12 定案)
# 话题 /supervisor/alerts @50Hz; supervisor 死亡 = 话题静默 (订阅方 20ms 级发现)。
#
# 原则: 错误处理在内部, 状态在此暴露 —— 单向观察窗, 不接任何指令
#       (清故障走总线状态机轴 fault_clear 握手; 控制器恢复靠自己重收目标)。
# 纪律: 本消息纯数值+定位名, 零写死容量 —— 三域统一"变长平行数组"模式
#       (JointState 同款, 加关节/加总线/加组件源 = 发布端多填一组, msg 永不改);
#       人话一律查 err_code_message(code) (码表住 include/unistackbot_interface/
#       error_codes.hpp, IkResult 先例, 加码不破 msg);
#       code=0 恒为"正常"; 0xFFFE 预留 = 该定位源静默;
#       各域内三数组严格等长 (违反 = 发布端 bug); CM 细节遥测走 ~/status 不重复。
#
# 三错误域 = 三个恢复语义世界 (用户 taxonomy, 与真机域四子包对应):
#   DEVICE  关节电机/驱动器自身 —— per-joint; 恢复 = 清故障握手+重新使能 (statemachine 轴)
#   BUS     总线通信            —— 恢复 = 自愈/重同步 (bus/serial 轴); 驱动器看门狗兜底安全终点
#   SYSTEM  控制器/软件域        —— 恢复 = 拒绝/降级/重规划 (控制器轴)

std_msgs/Header header      # 本板采样时刻 (心跳时序基准)

# ===== 域① DEVICE: 关节电机/驱动器 (per-joint; 仿真链恒空数组) =====
string[] device_locations   # 关节名 (按名对齐; 数量随形态)
uint16[] device_codes       # 每关节驱动器错误码 (协议轴 error_bits 映射; 任一非零 → FAULT)
uint64[] device_counts      # 每关节异常累计拍数 (趋势: 偶发 vs 持续)

# ===== 域② BUS: 总线通信 (per-端点; 仿真链恒空数组) =====
string[] bus_locations      # 总线端点标识 (如 "serial0"; 真机批次启用)
uint16[] bus_codes          # 超时/断链码 (偶发 → DEGRADED 级, 断链 → FAULT 级)
uint64[] bus_counts         # 异常累计拍数

# ===== 域③ SYSTEM: 控制器/软件域 (per-组件源; 仿真链产 cm/js/ee 三条) =====
string[] system_locations   # 组件定位名 ("cm" / "joint_states" / "ee_state" / 未来 "write_guard")
uint16[] system_codes       # 链上软件域错误码 (js 断流 = FAULT 级, 其余现行 = DEGRADED 级)
uint64[] system_counts      # 异常累计拍数

# ===== 系统聚合态 (上层判断"能不能派活"的一等结论, 唯一需 switch 的量) =====
uint8 state
uint8 STATE_UNKNOWN = 0     # 初始/源数据不足 (如 /joint_states 从未收到)
uint8 STATE_OK = 1          # 三域全零码
uint8 STATE_DEGRADED = 2    # 可用但降质 —— 可继续, 建议保守
uint8 STATE_FAULT = 3       # 不可派活 —— 停派/等恢复
# 聚合规则 (worst-of(err_code_severity), 唯一裁决处 = supervisor; 宁误停不误跑):
#   任一 DEVICE 非零码 → FAULT (设备错不经降级)
#   任一定位码 = 0xFFFE (源静默) → FAULT
#   全零码但核心源数据不足 → UNKNOWN
