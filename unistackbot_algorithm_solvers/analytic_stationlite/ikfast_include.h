#ifndef UNISTACKBOT_ALGORITHM__ANALYTIC_STATIONLITE_IKFAST_INCLUDE_H_
#define UNISTACKBOT_ALGORITHM__ANALYTIC_STATIONLITE_IKFAST_INCLUDE_H_

// vendored ikfast.h (OpenRAVE 生成配套) 的消费方包装:
// 1. IKFAST_HAS_LIBRARY: 启用 IkReal typedef 与 IkSolutionList 模板区 ——
//    不开则整个库区被预处理器裁掉 ("IkReal does not name a type" 的根因, 2026-09-22)
// 2. 告警抑制: ikfast.h 91/185 行函数返回值 const 的固有警告 (-Wignored-qualifiers)。
//    vendored 文件零改动, 再生成后无需重新打补丁。
// 注: ComputeIk/ComputeFk 原型不在 ikfast.h —— 消费方 (analytic_stationlite.cpp)
//     自行以 extern "C" 声明, 与 analytic_stationlite_ikfast.cpp 的定义链接一致。
// 本 TU 不定义 IKFAST_NAMESPACE (适配层用全局 IkReal); 生成 TU 以
// -DIKFAST_NAMESPACE=stationlite_ikfast 编译 (见 CMake)。
#define IKFAST_HAS_LIBRARY
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#include "ikfast.h"
#pragma GCC diagnostic pop

#endif  // UNISTACKBOT_ALGORITHM__ANALYTIC_STATIONLITE_IKFAST_INCLUDE_H_
