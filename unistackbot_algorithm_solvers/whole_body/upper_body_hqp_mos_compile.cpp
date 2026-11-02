// 编译哨兵 (2026-11-02 审核发现): upper_body_hqp_mos.hpp 是算法团队主模板但零编译覆盖
// (头文件不被任何 TU include = 语法/依赖烂了无人知)。本 TU 只 include, 进常规构建。
#include "whole_body/upper_body_hqp_mos.hpp"
