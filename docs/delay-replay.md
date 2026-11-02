# 延迟重放实验

本版新增 `delay_replay` 相机中继，并为控制器增加输入 QoS、可关闭终点停车及首次停止原因锁定。组长的根 README 保持原样。
已有 Ubuntu/ROS 2/Gazebo 试验及日志核对，见 [本阶段证据](../results/delay-replay-2026-10-30/README.md)。当前有单次接触案例，同版本重复对照仍待补齐。

## 数据与时间规则

链路：`/camera/image_raw -> delay_replay -> /attack/image_raw -> visual_baseline`。
仅基线节点发布速度命令，中继不订阅 LiDAR，也不改变场景。

收到时间戳为 t 的图像时，选择原始时间戳不晚于 t-D 的最新缓冲帧。
输出保留当前输入的消息头，图像内容来自历史帧；历史原始时间另存日志。
这里 D 使用相机消息头的仿真时间，不是墙上时间；需确认相机使用仿真时间戳。
按新输入帧触发输出，没有输入就不会继续伪造新帧。

- `start_s`：相对于中继收到第一帧的时间，**不是机器人开始行驶的时间**。
- `duration_s`：攻击窗口长度，开始包含、结束不包含；参数在启动时读取，修改后需重启。
- 默认 `start_s=0`，缓冲就绪后开始重放；默认窗口只有 60 仿真秒。
- 缓冲不足：实时转发并记录 `BUFFER_NOT_READY`，不能计作有效攻击。
- `delay_s=0`：实时转发，记录 `ZERO_DELAY`，作为经过同一路径的对照。
- 重复时间戳：不输出；时间倒退或图像格式/坐标系变化：清空缓冲并重新计时。
- 每次实验重启仿真、中继和控制器。乱序帧也会被当成时间倒退，查看 reset 日志。
- 最多缓存 300 帧、256 MiB 图像载荷，超限淘汰最旧帧；该限额不含 Python 对象及 DDS 内存。

帧率不规则时实际延迟可能大于 D。例如 t=1.6、D=1，若历史帧只有 0、0.4、0.9 秒，会选择 0.4 秒，实际延迟 1.2 秒。必须报告实际延迟分布。

## Ubuntu 构建和测试

把本目录完整复制到 Ubuntu 本地文件系统，进入仓库根目录。先按 [基线指南](baseline.md) 准备依赖。

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
bash scripts/build_baseline.sh
PYTHONPATH="$PWD/src/tb3_visual_baseline" python3 -m unittest discover -s src/tb3_visual_baseline/test -v
source install/setup.bash
export ROS_DOMAIN_ID=33
```

## 当前场景与运行

当前统一使用 `configs/baseline_white_line.yaml`，速度 1.0 m/s、阈值 0.4、终点停车关闭。不要使用旧 `baseline.yaml` 的默认速度和终点条件替代本阶段实验配置。
每轮先停止旧控制器、中继与 Gazebo，再在仓库根目录分三个终端运行：

终端 A：

```bash
ROS_DOMAIN_ID=33 bash scripts/start_scene.sh
```

等待 `SCENE_READY`，确认原点处机器人及房子、白线和红方块均正常显示。

终端 B：

```bash
# 0 秒中继对照；延迟组仅将 0.0 改为 0.5 或 1.0
ROS_DOMAIN_ID=33 bash scripts/run_relay.sh 0.0
```

0 秒组等待 `ZERO_DELAY`，延迟组等待 `REPLAY`，再运行终端 C：

```bash
ROS_DOMAIN_ID=33 bash scripts/run_relay_baseline.sh
```

上述脚本自动加载 ROS 环境、依赖工作区和本仓库 install。中继脚本将 duration_s 设为 600 仿真秒；默认 YAML 的 60 秒仅适用于自行调用节点时。
窗口从中继第一帧起算，必须确认覆盖运动阶段，等待太久应重启中继。所有组采用相同准备流程，记录实际运行配置和两份 run ID。
控制器应显示 `speed=1.0; red_threshold=0.4; goal_stop=False; sensor_timeout=2.0`。

[固定基线指南](experiment-baseline.md)包含依赖工作区、共享目录部署及手工启动方式。已有旧版本试验只作为开发记录，后续正式比较需同版本重复对照；本版不提供行驶中攻击的精确同步触发信号。

## 结果验收

中继每次启动写入 `~/replay-results/<UTC-run-id>/config.json` 和 `frames.csv`。
基线日志仍在 `~/baseline-results`，记录两者 run ID 的对应关系。
中继 CSV 包含输入时间、所选历史帧时间、实际延迟、阶段、缓冲重置及缓存用量。

1. 0 秒组的输出图像和时间戳应与输入一致，并复现正常停车。
2. 攻击组应出现 `REPLAY`，实际延迟不小于设定值；检查是否发生缓冲不足或重置。
3. 恢复后应出现 `AFTER_ATTACK` 并转发当前帧；可用短窗口单独检查，别混入正式长窗口试验。
4. 输入停止时输出也停止，基线原有超时逻辑应继续工作；需在 Ubuntu 联调验证。
5. 比较各组首次 `OBSTACLE_STOP` 的仿真时刻、最终前向位移及人工观察，保留所有失败和异常运行。

相同场景中有障碍时，历史帧里也可能有较小的障碍；该方法使视觉信息滞后，并不保证完全隐藏障碍。
当前配置的目标速度为 1.0 m/s；速度乘以延迟仅可作运动量估算，不能代替实测停车或接触数据。
现有日志的 distance_m 是前向位移，不是障碍净距。碰撞、最小净距和实际制动延迟仍需另外测量。
停车状态锁定，攻击结束不会让已停车的机器人自动恢复行驶。

原始 CSV 可使用 `scripts/audit_replay_run.py` 离线核对，具体命令见证据目录。脚本按控制日志图像时间戳与中继输入时间戳关联，不把 `REPLAY` 状态行的 elapsed 当成实际帧延迟。
