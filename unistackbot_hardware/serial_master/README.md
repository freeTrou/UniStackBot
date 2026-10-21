# serial_master — 串口总线骨架（包 unistackbot_serial）

> 2026-09-24 切片 1 步 3+4 落地。设计文档 `docs/bus/rs485_master_design.md`（§3 总线循环/时隙/追帧纪律）。
> R1-7A 七轴臂（485@6M）第一消费者。

## 五件套（全部零 ROS include，g++ 直编即测）

| 组件 | 文件 | 说明 |
|---|---|---|
| `IoTransport` | `io_transport.hpp` | 字节管道接口（read/write/bytes_available/close，全非阻塞） |
| `FakeTransport` | `fake_transport.hpp` | 假总线：写入字节交 responder（假电机）→ 应答入 RX 队列；故障注入旋钮（丢应答/坏字节/分片） |
| `TermiosTransport` | `termios_transport.*` | Linux 真串口：8N1 raw·BOTHER 任意波特·TIOCEXCL 独占·FIONREAD；kernel ABI 手工复刻（glibc/asm 头撞名的经典解法） |
| `Framer` | `framer.*` | 字节流→帧提取：帧头同步+定长切帧+跨调用拼接+坏帧重同步；CRC 归 decode；命令回显免疫（FE vs FC） |
| `SerialMaster` | `serial_master.*` | **MasterBase 实现**：总线线程 FIFO 85/核2、逐节点时隙调度、追帧跳过纪律、quick_stop 闩锁（锚定实测位）、**fault_clear 清故障闩锁**（2026-10-10：全节点请求→时隙发 kClear 帧→error_raw==0 确认握手→FAULT 粘性解除；无 FAULT 自熄）**动态容量**（2026-10-21 方案 R-a：节点表/交换载荷全运行期 vector，DynamicSpLatest 装配期定容，kMaxNodes 容量常量退役——节点数=实际电机/传感器表）、**安全机自建**（start 期按 `MasterConfig.statemachine` 经工厂 createNodeSafety per-node 实例——状态机是总线内部组件，请求生命周期机器自持，泵不代清）、SpLatest 交换、安全怠速（start 后未发命令=停机帧+看门狗位）、tx/rx 全遥测（含写失败/表外 id 分流）、可重启（stop 释放 transport、start 复位闩锁与泵态）、慢通道 kUnsupported |

## 组合根职责

```cpp
unistackbot_serial::registerToMasterFactory();   // 向 bus 工厂注册 "serial"
auto m = unistackbot_bus::createMaster("serial");
m->attach_transport(std::move(fake));            // IO 注入 (测试/FakeTransport; 缺省开真串口)
m->start(cfg);   // endpoint="fake" 或设备路径
// 组合模型 (2026-10-21): 协议内芯与状态机均 start 期按 cfg 节点表 per-node 经工厂
// 自建 (总线内部组件) —— cfg.protocol/statemachine 段级名 + 节点级覆写 (回退段级);
// set_protocol 注入式已退役。名字解析/kind 交叉校验 (执行器必填状态机, 传感器必空)
// 先于 io open —— 配置错误不碰硬件; 失败诊断带节点索引与总线 ID。
```

## 测试

| 测试 | cases | 覆盖 |
|---|---|---|
| `test_framer` | 9 | 噪声找帧/跨调用拼接/一批多帧/回显免疫/残尾恢复 |
| `test_serial_master` | 61 | 工厂注册/安全怠速/命令往返(位置 1e-3 精度)/安全机映射(unitree_im: READY·ENABLED·FAULT)/使能握手(锚定实测位+先验增益)/清故障握手(kClear 帧→error=0→kEnabled, ③b)/坏帧拒收/分片拼接/quick_stop(线上停机帧)/tx 失败遥测(FlakyWriteTransport)/重启(闩锁复位)/断流语义(停流=收敛到最后目标保持, 2026-10-10 统一)/组合模型装配预演(混型段级+节点覆写→串口正确拒/缺状态机拒/两级名皆空拒/ratio≠1拒/跨族拒/重复id拒/宇树段级简写零行为变化)/意图三件套(enable 显式路径+急停压意图+FAULT期意图失效(回READY不自动复电)+表外id免疫)/动态容量(方案R-a: 14节点顶格+id域自声明拒/重复id拒/比例域拒/跨族拒/两级名皆空拒, 2026-10-21) |

```bash
cd unistackbot_hardware/serial_master/test
# Framer
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_framer.cpp ../src/framer.cpp \
    ../../protocol/src/unitree_im_core.cpp -I../include -I../../protocol/include \
    -o /tmp/t_f && /tmp/t_f
# SerialMaster 全链 (FakeTransport + FakeMotor)
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_serial_master.cpp \
    ../src/serial_master.cpp ../src/framer.cpp ../src/termios_transport.cpp \
    ../../bus/src/master_factory.cpp ../../protocol/src/unitree_im_core.cpp \
    ../../protocol/src/dm_motor_core.cpp ../../protocol/src/protocol_factory.cpp \
    ../../statemachine/src/unitree_im_safety.cpp ../../statemachine/src/cia402_safety.cpp \
    ../../statemachine/src/dm_safety.cpp ../../statemachine/src/node_safety_factory.cpp \
    -I../include -I../../bus/include -I../../protocol/include -I../../statemachine/include \
    -I../../../unistackbot_common -o /tmp/test_serial_master && /tmp/test_serial_master


## 落地状态

- ✅ 切片 1 步 3（Framer + FakeTransport + 假从站）
- ✅ 切片 1 步 4（SerialMaster 总线循环 + 四态机全链）
- ✅ 切片 1 步 5 部分（TermiosTransport 编译通过，本机回环/真机待到货）
- ⬜ 切片 1 步 5 完成（SystemInterface 接 ros2_control）
