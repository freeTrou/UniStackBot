# original/ — 原始 URDF 留档 (回溯基线)

`arms/<robot>/original/` 存放**导入时点**的 URDF 原始文件 (vendor/community 上游原样,
未经本项目任何修改), 供后续改错时对照与回溯。

| 机型 | 提取自提交 | 上游来源 |
|---|---|---|
| piper | `94fea82` (新增 Piper 机械臂模型描述) | Piper 社区 ROS 包 |
| xarm7 | `e7e7201` (xarm7 机型接入 vendor) | UFACTORY xarm_ros2 (见 `LICENSE.xarm_ros2`) |

**本目录是死档案, 永不修改** —— 不参与构建 (xacro 不 include 它), 只作 diff 基准。

## 回溯方法

```bash
# 对照当前改了什么 (最常用)
diff -r unistackbot_description/arms/piper/original/ unistackbot_description/arms/piper/urdf/

# 单文件回看原始版
cat unistackbot_description/arms/piper/original/piper_macro.xacro

# 完整回滚到某个历史时点 (git 才是真版本管理, 本目录只是可见化基线)
git log --oneline -- unistackbot_description/arms/piper/urdf/
git show <sha>:unistackbot_description/arms/piper/urdf/piper_macro.xacro
git checkout <sha> -- unistackbot_description/arms/piper/urdf/
```

注意: original/ 只覆盖 urdf/ 描述文件。meshes 从未修改 (无需留档);
`config/kinematics`、`ik/` 等其他资产用 git 历史回溯。
