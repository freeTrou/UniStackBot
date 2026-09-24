# canfd_master — SocketCAN (CAN FD) 主站后端

**状态**：占位，未动工（2026-09-24 立目录）。**设计已成文**：`docs/bus/socketcan_master_design.md`。

## 要点（设计定稿摘录）

- SocketCAN 内核帧队列（`sockaddr_can`），CAN FD 帧；主站是交换契约后面的可插拔后端之一
- **确定性来源与 EtherCAT 不同**：CAN 仲裁制（非调度制）——1kHz 靠负载预算不靠配置；发帧策略 = 命令事件驱动 + 反馈轮询
- **看门狗自己造**：CAN 无 WKC——心跳超时检测 + 驱动器侧失能保护（无硬件看门狗时的软件防线）
- **时间戳降级**：无 DC，收帧级时间戳（延迟档案比 EtherCAT 宽一档，训练侧引用需记账）；MCU 侧 timesync 锚点表方案见 `docs/bus/timesync_design.md`
- Piper 真机 CAN 驱动预期落位于此（`piper_ros2_control.xacro` 的 device/baudrate/loop_rate 参数已预留）

## 契约

与其他主站共用 master.hpp 五要素（父类已代码化: `unistackbot_hardware/bus/` 的 MasterBase; `docs/architecture/hardware_framework_design.md` §4）；
Piper CAN 协议状态机映射为中立词汇；与 ethercat_master 结构平行。
