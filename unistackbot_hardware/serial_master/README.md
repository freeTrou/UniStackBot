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
| `SerialMaster` | `serial_master.*` | **MasterBase 实现**：总线线程 FIFO 85/核2、逐节点时隙调度、追帧跳过纪律、quick_stop 闩锁（锚定实测位）、**fault_clear 清故障闩锁**（2026-10-10：全节点请求→时隙发 kClear 帧→error_raw==0 确认握手→FAULT 粘性解除；无 FAULT 自熄）、**安全机自建**（start 期按 `MasterConfig.statemachine` 经工厂 createNodeSafety per-node 实例——状态机是总线内部组件，请求生命周期机器自持，泵不代清）、SpLatest 交换、安全怠速（start 后未发命令=停机帧+看门狗位）、tx/rx 全遥测（含写失败/表外 id 分流）、可重启（stop 释放 transport、start 复位闩锁与泵态）、慢通道 kUnsupported |

## 组合根职责

```cpp
unistackbot_serial::registerToMasterFactory();   // 向 bus 工厂注册 "serial"
auto m = unistackbot_bus::createMaster("serial");
m->attach_transport(std::move(fake));            // 测试/智能 dongle 注入 (可选)
m->start(cfg);                                   // endpoint="fake" 或设备路径
```

## 测试

| 测试 | cases | 覆盖 |
|---|---|---|
| `test_framer` | 9 | 噪声找帧/跨调用拼接/一批多帧/回显免疫/残尾恢复 |
| `test_serial_master` | 40 | 工厂注册/安全怠速/命令往返(位置 1e-3 精度)/安全机映射(unitree_im: READY·ENABLED·FAULT)/使能握手(锚定实测位+先验增益)/清故障握手(kClear 帧→error=0→kEnabled, ③b)/坏帧拒收/分片拼接/quick_stop(线上停机帧)/tx 失败遥测(FlakyWriteTransport)/重启(闩锁复位)/断流语义(停流=收敛到最后目标保持, 2026-10-10 统一) |

```bash
cd unistackbot_hardware/serial_master/test
# Framer
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_framer.cpp ../src/framer.cpp \
    ../../protocol/src/unitree_im_core.cpp -I../include -I../../protocol/include \
    -o /tmp/t_f && /tmp/t_f
# SerialMaster 全链 (FakeTransport + FakeMotor)
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_serial_master.cpp \
    ../src/serial_master.cpp ../src/framer.cpp ../src/termios_transport.cpp \
    ../../protocol/src/unitree_im_core.cpp ../../protocol/src/protocol_factory.cpp \
    ../../statemachine/src/unitree_im_safety.cpp \
    ../../statemachine/src/cia402_safety.cpp \
    ../../statemachine/src/node_safety_factory.cpp \
    ../../bus/src/master_factory.cpp \
    -I../include -I../../protocol/include -I../../statemachine/include \
    -I../../bus/include -I../../../unistackbot_common -pthread \
    -o /tmp/t_sm && /tmp/t_sm
```

## 落地状态

- ✅ 切片 1 步 3（Framer + FakeTransport + 假从站）
- ✅ 切片 1 步 4（SerialMaster 总线循环 + 四态机全链）
- ✅ 切片 1 步 5 部分（TermiosTransport 编译通过，本机回环/真机待到货）
- ⬜ 切片 1 步 5 完成（SystemInterface 接 ros2_control）
