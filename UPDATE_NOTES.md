# 本版更新内容

## 延迟重放与仿真核对（实验 2026-10-30，整理 2026-11-02）

- 新增独立 `delay_replay` 相机中继：按输入帧的仿真时间选择历史图像，保留当前消息头。
- 支持延迟、开始时间、持续时间、缓存帧数和载荷内存上限；记录逐帧实际延迟及攻击状态。
- 缓冲不足时实时转发并标记；重复时间戳不输出，时间倒退或格式变化时清空缓冲。
- 中继输出和控制器输入使用 reliable、深度 1；新增逐帧耗时及双路相机检查工具。
- 一条命令加载房子、道路、白线和 X=4.0 m 的红方块。当前实验统一速度 **1.0 m/s**、视觉阈值 **0.4**、终点停车 **关闭**，首次停止原因锁定。
- 最新 1 秒试验配置和原始日志已核对：运动至首次停车的 47 个匹配图像帧实际重放延迟为 **1.022–1.353 仿真秒**，平均 **1.099 秒**。首次停止为 `OBSTACLE_STOP`，触发位移约 **4.000 m**。
- 停车前最后一个采样仍记录前进指令 1.0 m/s、里程计速度约 -0.00164 m/s；随后控制指令归零，停车起的 17,185 条记录均为零速度指令。操作者报告接触方块并完全停住；日志本身不提供精确接触时刻。
- 原始日志、配置、SHA-256 和可重算核对结果见 [本阶段证据](results/delay-replay-2026-10-30/README.md)。旧 0.5 秒及旧 1 秒试验分开归档；旧 1 秒试验首先触发终点停车。
- 运行步骤见 [当前固定场景](docs/experiment-baseline.md)；机制与核对方法见 [延迟重放指南](docs/delay-replay.md)。
- 本地测试及环境限制记录在 [验证记录](results/delay-replay-2026-10-30/validation/README.md)。实际 Gazebo 运行由操作者在 Ubuntu 完成；本次 Windows 检查不等同于重新运行 ROS/Gazebo。
- 已完成延迟注入和单次接触案例核对。尚未完成同版本重复对照、自动接触/净距测量及 LiDAR 防御评估，不报告攻击成功率。

以下为此前已交付的视觉基线及其验证记录。

The sections below retain the earlier visual-baseline delivery and its evidence. The shared project proposal remains in [README.md](README.md).

## Implementation delivered

With Codex assistance, I prepared and validated the first camera-based forward-navigation baseline for TurtleBot3 Waffle Pi:

- Added the [`tb3_visual_baseline` ROS 2 package](src/tb3_visual_baseline), including an OpenCV HSV detector for a red obstacle in the central camera region.
- Connected camera images and odometry to a controller that drives forward, stops at the configured endpoint, and latches a stop when the visual obstacle threshold is reached.
- Added observation mode and a debug-image output to inspect detection before enabling motion.
- Added sensor timeout and execution safeguards, plus per-run configuration and CSV logging for states, visual ratios, displacement, commands and odometry velocity.
- Supplied [baseline parameters](configs/baseline.yaml), a [red-box simulation model](worlds/red_obstacle.sdf), [build/run and log-summary scripts](scripts), and [pinned TurtleBot3 dependency revisions](configs/turtlebot3.repos).

The implementation currently combines visual perception and control in one node. Its image topic is configurable for later integration. LiDAR does not control this baseline.

## Environment setup and troubleshooting

I operated the Ubuntu virtual machine and verified Ubuntu 24.04.4 LTS, ROS 2 Jazzy, Gazebo Sim 8.15.0 (Harmonic), and the Waffle Pi model. The VM used 4 virtual CPUs and 8 GiB RAM.

I investigated Gazebo flickering and blank rqt image windows with assistance. Software rendering resolved the reported Gazebo display issue; a fresh rqt configuration and X11 backend restored image viewing. I documented these steps together with scene reset, obstacle placement and execution instructions in [the baseline guide](docs/baseline.md).

## Tests and simulation trials completed

- Ran the unit tests in Ubuntu and supplied a terminal transcript showing **10 tests passed**: 7 control tests and 3 synthetic-image tests.
- Ran clear-route trials with a 1 m endpoint. The two retained runs reached `GOAL_REACHED`, with final logged forward displacements of approximately 1.0004 m and 1.0021 m.
- Ran four moving approaches to the fixed red obstacle. All four reached `OBSTACLE_STOP`, with final logged forward displacements of approximately 0.9345 m, 0.9339 m, 0.9347 m and 0.9440 m.
- Reported no contact for the moving obstacle approaches and supplied screenshots for three of them. All six main runs ended with a logged forward odometry velocity of 0.0 m/s.

These displacement values are measured from the robot's initial pose; they are not obstacle-clearance measurements. These were preliminary fixed-scene development checks.

## Evidence and handoff

I exported the Ubuntu run data and supplied observations and screenshots. With Codex assistance, the submission was organized into:

- [The trial evidence index](results/baseline-2026-10-27/README.md), linking results, screenshots and evidence limitations.
- [All 12 original timestamped run directories](results/baseline-2026-10-27/runs), including observation-only and immediate-stop runs.
- [An aggregate CSV summary](results/baseline-2026-10-27/summary.csv), per-run summaries and a [source manifest](results/baseline-2026-10-27/source-manifest.json).
- [An AI assistance and human validation declaration](docs/ai-usage.md), distinguishing assisted implementation/documentation from the tests and simulation operations I performed.

The evidence index also records missing screenshots and the saved-copy discrepancy. The earlier saved CSV and the full export of the supplementary run are two snapshots of one trial, not separate trials.

## Historical baseline completion boundary (2026-10-27)

This version delivers the visual-navigation baseline and its preliminary validation evidence. Camera-input attacks, LiDAR cross-sensor verification, automated collision/clearance and latency evaluation, and the formal attack/defense comparison have not been implemented or validated in this version.

