# docs/ — 设计文档与课程笔记

分类（2026-09-21）：`architecture/` 框架总纲 · `bus/` 总线与硬件 · `sim/` 仿真与测试 · `guides/` 规范与方法论 · `control_course/` 课程笔记。

| 文档 | 一句话 | 备注 |
|---|---|---|
| [architecture/hardware_framework_design.md](architecture/hardware_framework_design.md) | 真机层与通用框架设计总纲（master.hpp 契约/形态盲/§14 边界） | 讨论中未收口 |
| [architecture/real_hardware_architecture.md](architecture/real_hardware_architecture.md) | **真机架构定稿视图**（多总线×多臂: 七原则/四轴分层/调度阶梯实测/双臂形态/接口统一/决策索引） | 2026-09-24 分叉自立, 讨论见母文档 |
| [architecture/rt_software_architecture.md](architecture/rt_software_architecture.md) | RT 软件架构宣言（七支柱） | RT 代码纪律依据 |
| [bus/socketcan_master_design.md](bus/socketcan_master_design.md) | SocketCAN CAN FD 主站完整设计 | 对 master.hpp 契约 |
| [bus/ethercat_master_design.md](bus/ethercat_master_design.md) | IgH EtherCAT 主站完整设计（1kHz+DC） | 与 SocketCAN 结构平行 |
| [bus/timesync_design.md](bus/timesync_design.md) | MCU↔主机时间同步两帧协议 | v1.9 USB 转串口部署版 |
| [bus/udp_internal_bus_argument.md](bus/udp_internal_bus_argument.md) | 内部总线选型论证（插值器以下不用 UDP） | CAN FD 定版依据 |
| [guides/cpp_style_guide.md](guides/cpp_style_guide.md) | **全仓 C++ 风格规范（binding）** | 新代码必读 |
| [guides/linux_rt_guide.md](guides/linux_rt_guide.md) | Linux RT 调优指南（PREEMPT_RT/isolcpus/cyclictest） | 第二部分=RT 代码纪律 |
| [guides/ik_validation_playbook.md](guides/ik_validation_playbook.md) | 换臂/换 IK 求解器六阶段验证方法论 | 参数审计表+验收模板 |
| [sim/sim_environment_and_test_plan.md](sim/sim_environment_and_test_plan.md) | 三链仿真体系：现状/互补定位/契约/测试矩阵/批次 | 仿真体系单一事实源 |
| [sim/chain_interface_dictionary.md](sim/chain_interface_dictionary.md) | 链上接口词典：话题/字段/频率/查看方式/服务/控制器状态 | 写上层算法与验收时查 |
| [sim/acceptance_report_template.md](sim/acceptance_report_template.md) | 机型验收报告模板（A-G 手填版） | run_acceptance 就位前用 |
| [sim/sim_validation_pipeline.md](sim/sim_validation_pipeline.md) | 机型接入→验收归档端到端流程（A-G 门禁） | 可复用资产 |
| [control_course/](control_course/) | 控制理论课程笔记（传递函数→三环级联） | 背景知识，非代码文档 |
