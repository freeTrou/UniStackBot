# unistackbot_hardware

真机总线主站**容器目录**（2026-09-24 起：**非 ROS 包**——无 package.xml/CMakeLists，
colcon 自动忽略，同 `unistackbot_common` 先例；代码动工时再复包）。

真机域容器 = **四子包 + 三骨架目录**（2026-09-24 三轴定形: 协议/状态机/总线同款
父类+子类+工厂组织, 仿 sim_control 容器先例——目录归家, 包名不变）：

```
unistackbot_hardware/              (容器; 内含三个 colcon 包)
  serial_master/     # ★包 unistackbot_serial: 串口骨架五件套已落地 (SerialMaster=MasterBase 实现
                     #  泵+时隙+追帧+quick_stop+FakeTransport 全链 25 cases 绿 + Framer 9 cases 绿
                     #  + TermiosTransport 编译过待真机) —— 见其 README
  ethercat_master/   # IgH ecrt EtherCAT (1kHz + DC) — 设计已成文
  canfd_master/      # SocketCAN CAN FD (Piper 真机预期落位) — 设计已成文
  protocol/          # ★包 unistackbot_protocol: codec 轴 (ProtocolCore 父类 + UnitreeImCore
                     #  子类 + 工厂; 厂商甲骨文 256 cases 绿)
  statemachine/      # ★包 unistackbot_statemachine: 状态翻译轴 (StateTranslator 父类
                     #  + UnitreeImTranslator 子类 + 工厂; 9 cases 绿; 依赖 protocol 包)
  bus/               # ★包 unistackbot_bus: 总线主站轴 (MasterBase 父类=五要素最小集代码化,
                     #  纯交换零线程可见 + 工厂; 注册表空占位——serial 随泵骨架落地;
                     #  BusCommand/BusState 自持零 ROS; 依赖 protocol+statemachine)
```

三主站共用 MasterBase 父类（`bus/` 包, 五要素已代码化; 设计 §4），
向上只暴露 `unistackbot_interface` 的 RobotFeedback/RobotCommand + 中立状态词汇。
Piper 真机参数预埋仍在：`piper_ros2_control.xacro`（description 包）的 `device`/`baudrate`/`loop_rate`。

## 设计先行

实现前先读设计文档（均按 MasterBase 五要素契约撰写，接口已代码化于 bus/ 包）：

- `docs/architecture/hardware_framework_design.md` —— 分层架构、线程模型、协议后端契约、v3 语义状态机
- `docs/bus/socketcan_master_design.md` —— SocketCAN CAN FD 主站后端
- `docs/bus/ethercat_master_design.md` —— IgH ecrt EtherCAT 主站后端（1 kHz + DC）
- `docs/bus/rs485_master_design.md` —— 串口主站（1.0 立稿，R1-7A 实例）
