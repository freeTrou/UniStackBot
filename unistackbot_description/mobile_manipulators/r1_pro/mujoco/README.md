# r1_pro MuJoCo 资产档案

来源: 官方 2026 g1z URDF (`original/`) 程序化提取 → 手写策展 MJCF。
18 执行器 = torso×4 (kp 800) + 双臂×7 (arm1-2 kp 800 / arm3-4 kp 500 / arm5-7 kp 300),
`implicitfast` 积分器, meshdir 直引 `../meshes/` collision STL。

## 单块纪律 (2026-10-14 双窗口案例, 勿回退)

`<ros2_control>` 必须一机型一块。曾经 arms/torso 双块（四件式初版）导致:

1. mujoco 插件按组件加载两次，而它是**进程单例**设计——渲染线程、`/clock`、
   reset/step 服务都由组件创建；
2. 第二组件（torso 块）找不到自己块里的 `mujoco_model` 参数 → fallback 等
   `/mujoco_robot_description` 话题 → **120 s 超时** → 组件初始化失败 → CM abort；
3. 用户可见症状 = **两个窗口**（有模型的 `MuJoCo : r1_pro` = 第一组件；空白的
   `MuJoCo` = 第二组件等模型期间已起的 UI），进程崩溃时一起消失。

控制器编组（js_arms/js_torso）不靠块拆分——归 `r1_pro_controllers.yaml` 的
`joints` 显式列表（JointStreamController 多实例参数，2026-10-14 定形）。

## 窗口语义

- `headless:=false`（默认）: 单窗口 `MuJoCo : <机型>`；`headless:=true`: 无渲染窗
  （基准/录制用，URDF 硬件参数传递，apt 0.1.2 无环境变量支持）。
- 上游主循环 60fps 渲染+睡眠结构带节拍税（分帧抖动），见 RT 基准报告
  `test/results/`（上游 PR 修复在途）。
