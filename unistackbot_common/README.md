# unistackbot_common

自研组件库：**不是 ROS 包**（无 package.xml / CMakeLists，colcon 自动忽略），纯代码存放层，保持可在非 ROS 环境（RT 主站线程、单元测试、ARM 交叉编译）中直接复用。每个组件一个子文件夹 = 文档 + 实现 + 测试三件套。

| 组件 | 一句话 | 语义 | 状态 |
|---|---|---|---|
| [`sp_latest/`](sp_latest/README.md) | 双缓冲覆盖写 / 取最新 | 最新 **1** 个（值通道） | 已交付（外部评审 7 轮） |
| [`sp_ring/`](sp_ring/README.md) | SPSC 无锁环形队列 | 逐条必达（事件通道），满拒新 | 已交付 |
| [`mpsc_ring/`](mpsc_ring/README.md) | 多写单读覆盖式无锁环 | 丢旧保新（日志 / 滑动窗口） | 已交付（外部评审 6 轮，TSAN 零竞争） |
| [`ulog/`](ulog/README.md) | 高性能异步日志 | 前端无锁入队，后端单线程双 sink | 已交付（29 项断言 + 三 sanitizer） |

## 测试

不进 colcon，组件目录下 g++ 直接编译运行（规范命令见 [`ulog/README.md`](ulog/README.md)，其余同型）：

```bash
cd <component>
g++ -std=c++17 -O2 -pthread -Wall -Wextra -Wconversion -I.. test_<component>.cpp -o test_<component> && ./test_<component>
```

测试二进制已在仓库根 `.gitignore`，勿提交。
