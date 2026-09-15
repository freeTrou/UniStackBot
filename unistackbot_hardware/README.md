# unistackbot_hardware

真机硬件抽象 / 驱动通信层（hardware_interface、pluginlib、serial/CAN）。

## 现状

**空骨架**（`src/placeholder.cpp`）。依赖与参数已按真机 Piper CAN 驱动预留：`piper_ros2_control.xacro`（description 包）里预埋了 `device` / `baudrate` / `loop_rate` 参数。

## 设计先行

实现前先读设计文档（均已按 `master.hpp` 五要素契约撰写，讨论中未收口）：

- `docs/hardware_framework_design.md` —— 分层架构、线程模型、协议后端契约、v3 语义状态机
- `docs/socketcan_master_design.md` —— SocketCAN CAN FD 主站后端
- `docs/ethercat_master_design.md` —— IgH ecrt EtherCAT 主站后端（1 kHz + DC）
