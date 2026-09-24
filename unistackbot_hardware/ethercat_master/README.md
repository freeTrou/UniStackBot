# ethercat_master — EtherCAT 主站后端

**状态**：占位，未动工（2026-09-24 立目录）。**设计已成文**：`docs/bus/ethercat_master_design.md`。

## 要点（设计定稿摘录）

- IgH ecrt **用户态** API，主站 RT 线程内跑总线循环（ioctl 每拍），PREEMPT_RT 上 1kHz 工业验证过；不上内核态开发
- **DC 分布式时钟**：循环内每拍 `application_time` + `sync_slave_clocks`——三总线里唯一有全网 µs 级时钟的
- 错误面八类（WKC 每拍每从站归因 / 断链定位到段 / AL 状态码 / ESC 端口计数器 / SM watchdog / DC 偏差 / mailbox abort），处理分级与 CAN 同构
- 崩溃语义：进程死 → fd 关 → master 自动释放 → 帧停 → 驱动器看门狗跳，安全链天然不断
- 部署成本（DKMS / master 实例 / 网卡绑定）写进 bring-up 文档

## 契约

与其他主站共用 master.hpp 五要素（父类已代码化: `unistackbot_hardware/bus/` 的 MasterBase; `docs/architecture/hardware_framework_design.md` §4）；
CiA402 状态机映射为中立词汇；与 socketcan_master 结构平行、共享协议层与三通道结构。
