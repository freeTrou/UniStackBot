# unistackbot_common

自研组件库：**不是 ROS 包**（无 package.xml / CMakeLists，colcon 自动忽略），纯代码存放层，保持可在非 ROS 环境（RT 主站线程、单元测试、ARM 交叉编译）中直接复用。每个组件一个子文件夹 = 文档 + 实现 + 测试三件套。

| 组件 | 一句话 | 语义 | 状态 |
|---|---|---|---|
| [`sp_latest/`](sp_latest/README.md) | 双缓冲覆盖写 / 取最新 | 最新 **1** 个（值通道） | 已交付（外部评审 7 轮） |
| [`sp_ring/`](sp_ring/README.md) | SPSC 无锁环形队列 | 逐条必达（事件通道），满拒新 | 已交付 |
| [`mpsc_ring/`](mpsc_ring/README.md) | 多写单读覆盖式无锁环 | 丢旧保新（日志 / 滑动窗口） | 已交付（外部评审 6 轮，TSAN 零竞争） |
| [`ulog/`](ulog/README.md) | 高性能异步日志 | 前端无锁入队，后端单线程双 sink | 已交付（29 项断言 + 三 sanitizer） |
| [`ruckig/`](ruckig/README.md) | vendored pantor/ruckig v0.14 + `otg_stream.hpp` 防御封装 | RT 轨迹整形（已知数值风险有防线；稳态零 malloc） | 已交付（压测套件实测触发原生 -110） |
| [`stale_watch/`](stale_watch/README.md) | 陈旧看门狗 | IDLE/LIVE/STALE 三态，周期计数制零 syscall（"无流=待命≠断流"） | 已交付（JS/CM 断流受控减速的判定原语） |
| [`rt_tune/`](rt_tune/README.md) | `RtTune` RT 线程调优参数化应用 | 亲和性/调度策略只作用于**自有线程**（CM 主线程走 yaml 预留接口，分工勿混） | 已交付（双线程 FIFO 基准 p99 8-11µs） |

## 测试

不进 colcon，组件目录下 g++ 直接编译运行（规范命令见 [`ulog/README.md`](ulog/README.md)，其余同型）：

```bash
cd <component>
g++ -std=c++17 -O2 -pthread -Wall -Wextra -Wconversion -I.. test_<component>.cpp -o test_<component> && ./test_<component>
```

测试二进制已在仓库根 `.gitignore`，勿提交。
