# unistackbot_gazebo

Gazebo 集成层：world + launch。两条链路，controller_manager 都活在仿真器的 ros2_control 插件里（无独立 ros2_control_node）。

| 链路 | 入口 | 状态 |
|---|---|---|
| Gazebo Sim (Fortress) | `ros2 launch unistackbot_gazebo piper_ign.launch.py`（`gui:=true\|false`） | **现行** |
| Gazebo Classic | `ros2 launch unistackbot_gazebo gazebo.launch.py` | EOL，留作参考 |

## 用法纪律

```bash
ros2 run unistackbot_gazebo gz_clean.sh   # 每次启动前必跑：清残留进程
```

ign server 常在 launch 关闭后存活并毒化下一次启动；且**一次只跑一套栈**（同名 controller_manager / robot_state_publisher 会互相污染）。

## 内置的坑规避（launch 里已写死，改动前先懂为什么）

- URDF 先压成**单行**再传：插件以 `--param robot_description:=<urdf>` 转发，rcl 解析器拒绝换行 → CM 起不来。
- 仿真器经**临时文件**（`-file`）拿 URDF，不走 `/robot_description` topic：iceoryx/SHM 下 TRANSIENT_LOCAL 锁存重发不可靠。
- **不覆盖** `CYCLONEDDS_URI`：本机 `~/cyclonedds.xml`（lo 绑定 + 单播 Peers）是 DDS 随机失联的修复配置。
- ign 链把 description 包的 share 根前置进 `IGN_GAZEBO_RESOURCE_PATH`：URDF→SDF 会把 `package://` 改写成 `model://`，Fortress 只认这个环境变量。

## 关键文件

`worlds/empty_ign.world`（`<world name>` 为 `piper_world`，须与 `sim_control_gz_node` 的 `world` 参数一致）、`worlds/empty.world`（Classic）、`scripts/gz_clean.sh`。
