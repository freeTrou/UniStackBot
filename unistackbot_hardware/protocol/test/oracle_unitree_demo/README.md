# 测试甲骨文 (test-only fixture)

厂商 IM6014 C++ demo 的 `MotorProtocol.{h,cpp}` 原样拷贝 (来源:
`/home/work/R1_7A_DOC/IM6014_C++_Demo_Cmake/`, 2026-09-24)。**仅被
`test_unitree_im.cpp` 编译引用作 golden 对拍**——不进任何生产代码/库目标;
生产实现 (`src/unitree_im.cpp`) 依协议文档独立编写, 与本夹具只在对拍测试中相遇。
