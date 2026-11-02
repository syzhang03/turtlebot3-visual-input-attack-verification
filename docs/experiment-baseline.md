# 后续实验固定基线（2026-10-30）

以下为当前场景和参数。已完成的开发试验与版本差异见 [实验记录](../results/delay-replay-2026-10-30/README.md)；同版本重复对照仍待补齐。

| 项目 | 固定值 |
|---|---|
| 场景 | `worlds/intersection-white-line.sdf`：房子、道路、白色停止线和一个红方块 |
| 场景生成位置 | X=0、Y=0、Z=0 |
| 机器人初始位置 | X=0、Y=0，沿 +X 方向；每轮重启场景 |
| 红方块中心（平面） | X=4.0 m、Y=0；场景已包含，无需单独生成 |
| 目标线速度 | **1.0 m/s**；直连、0 秒中继和所有延迟组统一，实际速度以基线 CSV 的 `odom_v` 为准 |
| 视觉阈值 | 0.4，红色轮廓面积占检测区域面积的比例 |
| 终点停车 | **关闭**，`enable_goal_stop=false`；保留的 `goal_distance=4.0` 不参与停车判断 |
| 传感器超时 | 2.0 秒墙钟时间 |
| 最大运动时间 | 120 秒仿真时间 |
| 控制配置 | `configs/baseline_white_line.yaml` |
| 控制输入 | 中继对照和攻击组均为 `/attack/image_raw` |
| 中继传输 | 中继输出与控制器图像接收均为 reliable，深度 1；已有 Ubuntu 运行日志，详见 [链路排查](relay-transport.md) |

保留已有文件名以兼容运行命令；`intersection-white-line.sdf` 中的方块现已恢复 X=4.0，并非先前的 X=3.4。场景文件 SHA-256：

`3A17682115EF1A864B6417B2A02FB39240C4EC65DDFBD845D8CE7C43043950DF`

## 试验顺序

1. 直连相机：用新的 1.0 m/s、0.4 条件重新运行并保存日志。
2. 0 秒中继对照：验证实时转发时仍可正常执行。
3. 0.5 秒和 1 秒延迟重放；先补齐同版本的对照与重复，再决定是否扩展延迟范围。

中继组之间只改 `delay_s`。速度、视觉阈值、位置、终点停车开关和超时均保持一致。
当前所有组统一关闭终点停车，并锁定首次停止原因。更新及版本说明见 [goal-stop.md](goal-stop.md)。正式对照需在新版条件下重新采集；旧版先触发 GOAL_REACHED 的试次保留，不改写成视觉停车成功。
20261030T143044.713472Z 是此前 **1.2 m/s** 的 0 秒中继运行：终端记录视觉停车触发位移 3.605 m，用户观察无接触并停住。它保留为原条件记录，不能替代 **1.0 m/s** 的新对照，也不能改写其 config.json 中的速度。
直接读相机的试验用于排查链路；正式延迟组以经过同一中继的 0 秒组作对照。
每组独立重复并保留全部结果，不能只保留成功停车的运行。

## 每轮启动

### 一条命令启动场景

停止旧控制器、中继和 Gazebo 后，只需运行：

```bash
bash ~/delay-replay-2026-10-30/scripts/start_scene.sh
```

它自动加载 ROS 环境、启动原点处的 Waffle Pi，并等待 Gazebo 服务创建房子、道路、白线和 X=4.0 m 的红方块。看到 `SCENE_READY` 后，在 Gazebo 确认机器人和场景均正常显示。
不需要再执行下面的终端 A 命令和终端 B 的 `ros_gz_sim create` 命令；后续中继和控制器仍分别启动。
此脚本不自动让机器人行驶，也不修改控制参数（固定配置为速度 1.0、阈值 0.4）。
终端保持打开，按 Ctrl+C 关闭本脚本启动的仿真。每轮复用同一条命令重新生成场景。

加载失败或超过 120 秒时会输出 `SCENE_FAILED` 并请求关闭本次仿真。脚本通过文件锁防止同一用户、同一 ROS_DOMAIN_ID 重复启动本脚本，但不能替代关闭此前手工启动的 Gazebo。
可用 `TB3_WS` 指定依赖工作区；默认 `~/turtlebot3_ws`。默认域 ID=33，默认软件渲染。

### 手工启动（备用）

先停止旧控制器和中继，再关闭旧 Gazebo。下面三个终端都从仓库根目录运行，使用 ROS_DOMAIN_ID=33。

终端 A：

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export ROS_DOMAIN_ID=33
export TURTLEBOT3_MODEL=waffle_pi
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch turtlebot3_gazebo empty_world.launch.py x_pose:=0.0 y_pose:=0.0
```

终端 B，等待 Gazebo 启动后：

```bash
cd ~/delay-replay-2026-10-30
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=33
ros2 run ros_gz_sim create -file "$PWD/worlds/intersection-white-line.sdf" -name boulder_intersection -x 0.0 -y 0.0 -z 0.0
```

确认场景生成成功后，在终端 B 启动 0 秒中继：

```bash
ros2 run tb3_visual_baseline delay_replay --ros-args \
  --params-file "$PWD/configs/delay_replay.yaml" \
  -p delay_s:=0.0 -p start_s:=0.0 -p duration_s:=600.0
```

0 秒组等待 `ZERO_DELAY`；延迟组将上面的 delay_s 改为本组值并等待 `REPLAY`，然后才运行控制器。
窗口从中继收到第一帧算起，不从机器人起步算起；准备时间过长需重启中继。

终端 C：

```bash
cd ~/delay-replay-2026-10-30
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=33
ros2 run tb3_visual_baseline baseline --ros-args \
  --params-file "$PWD/configs/baseline_white_line.yaml" \
  -p image_topic:=/attack/image_raw \
  -p image_reliability:=reliable \
  -p enabled:=true
```

如需排查原始相机，将最后命令的 image_topic 改为 `/camera/image_raw`，image_reliability 改为 `best_effort`，并标记为直连运行。

为避免旧命令覆盖速度，优先使用统一配置的脚本：

```bash
# 原始相机直连行驶（一个终端）
bash ~/delay-replay-2026-10-30/scripts/run_baseline.sh drive

# 中继组：先在终端 B 启动，参数为本组延迟（0.0、0.5 等）
bash ~/delay-replay-2026-10-30/scripts/run_relay.sh 0.0
# 等待 ZERO_DELAY 或 REPLAY 后，在终端 C 启动行驶
bash ~/delay-replay-2026-10-30/scripts/run_relay_baseline.sh
```

两种行驶脚本都从 `configs/baseline_white_line.yaml` 读取 speed=1.0、red_area_threshold=0.4，不再硬编码覆盖速度。
旧手工命令中的 `-p speed:=1.2` 会覆盖 YAML；如继续手工运行，删掉该项或改为 `-p speed:=1.0`。

## 每轮保存

- 记录中继和基线终端打印的两份日志目录，以及本轮延迟设置。
- 检查新运行的 config.json 中 speed=1.0、red_area_threshold=0.4、enable_goal_stop=false 和实际输入话题。
- 记录停车状态、是否实际停住、是否接触方块；截图与本次 run ID 对应。
- `OBSTACLE_STOP` 是视觉停车状态；`GOAL_REACHED` 是到达终点；`SENSOR_TIMEOUT`、`FAULT` 和人工终止单独标记，不当作视觉停车成功。
- 停车后保留约 3 秒日志，再停止控制器和中继。如已发生接触，立即结束并记录接触。
- 通过实际延迟日志验证攻击覆盖运动阶段；缓冲不足、重置和攻击结束后的片段不算有效重放。

`distance_m` 是前向位移，不是障碍净距。终端显示的是状态变化时的位移，不一定等于最终停车位移。
碰撞、最小净距及实际制动延迟没有自动评估器；不得仅依据位移或 `OBSTACLE_STOP` 声称无碰撞。
本会话已出现相机接收/处理超时，尚未完成根因排查；改变速度或阈值不能证明该问题已解决。
