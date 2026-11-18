# 本版更新内容

本阶段完成了 **TurtleBot3 仿真环境搭建、视觉停车基线和历史帧延迟重放攻击**，供组员继续开发和验证防御方法。

## 1. 搭建的环境

| 项目 | 内容 |
|---|---|
| 系统 | VMware 中的 Ubuntu 24.04.4 LTS，4 核 CPU、8 GiB 内存 |
| 机器人软件 | ROS 2 Jazzy、Gazebo Harmonic（实测 Gazebo Sim 8.15.0） |
| 机器人 | TurtleBot3 Waffle Pi，使用前置相机和里程计 |
| 场景 | 房子、道路、白色停止线和固定红色方块；红方块位于 X=4.0 m、Y=0 |
| 场景启动 | `scripts/start_scene.sh` 一次加载机器人与完整场景 |
| 当前统一参数 | 速度 **1.0 m/s**、红色面积阈值 **0.4**、关闭终点停车 |

场景和代码已放入仓库，组员不需要重新手工搭房子、画白线或放置方块。依赖版本记录在 [turtlebot3.repos](../../configs/turtlebot3.repos)。

## 2. 基线测试与攻击结果

视觉基线已跑通：机器人沿道路前进，根据相机中红色障碍物的面积比例决定停车。**无延迟时，小车可以停在白线处。**白线用于观察停车位置，当前控制器没有白线识别或循线算法。

本阶段采用 **历史帧延迟重放攻击**：在相机与控制器之间加入缓冲中继，让控制器持续看到若干秒之前的图像。障碍物在旧图像中显得更远，因此可能推迟视觉停车。

| 中继延迟设置 | 已完成试验的现场观察 |
|---|---|
| **0 秒** | 小车停在白线处，基线停车成功 |
| **0.5 秒** | 小车越过白线后停住，没有接触方块 |
| **1 秒** | 小车撞上红色方块，随后完全停住 |

表中白线位置和接触情况来自操作者观察。最新 1 秒试验的日志已核实延迟重放生效，并在触发 `OBSTACLE_STOP` 后持续输出零速度指令。不同试次的原始记录及版本差异见 [实验记录](../../results/delay-replay-2026-10-30/README.md)；本表描述已经观察到的现象，不表示每次运行都会得到完全相同的停车位置。

攻击链路如下：

```text
/camera/image_raw → delay_replay → /attack/image_raw → visual_baseline → /cmd_vel
```

延迟使用图像时间戳的**仿真时间**。中继选择不晚于“当前时间减去设定延迟”的最新历史帧，实际延迟会受到帧间隔影响。输出图像保留当前消息头，历史内容的真实时间另存日志；仅检查输出 header 的新旧不能识别这种内容延迟。

## 3. 组员如何复现

首次安装、下载分支和构建按 [复现与防御接入指南](../reproduce-and-defense.md) 完成。在 Ubuntu 本地目录构建后，每轮只需三个终端；以下约定仓库位于 `~/tb3-delay-replay`。

**终端 A：加载完整场景。**

```bash
cd ~/tb3-delay-replay
ROS_DOMAIN_ID=33 bash scripts/start_scene.sh
```

看到 `SCENE_READY` 后，确认机器人在起点，房子、白线和红方块正常显示。

**终端 B：启动中继，选择本轮延迟。**

```bash
cd ~/tb3-delay-replay
ROS_DOMAIN_ID=33 bash scripts/run_relay.sh 0.0
```

先用 `0.0` 复现无延迟基线；后续分别改为 `0.5`、`1.0`。无延迟等待 `ZERO_DELAY`，有延迟等待 `REPLAY`。

**终端 C：启动视觉控制器。**

```bash
cd ~/tb3-delay-replay
ROS_DOMAIN_ID=33 bash scripts/run_relay_baseline.sh
```

启动信息应显示 `speed=1.0; red_threshold=0.4; goal_stop=False; sensor_timeout=2.0`。
记录停车位置、是否越过白线、是否接触方块，并保存截图和两个日志目录。每轮结束后按顺序停止控制器、中继、场景，再从终端 A 重新开始；只改延迟，不改速度、阈值和障碍位置。

## 4. 在此基础上做防御

组员可直接复用当前场景、相机攻击中继和视觉控制器。防御节点可以读取 **受攻击图像 `/attack/image_raw`、未被本攻击修改的 LiDAR `/scan` 和里程计 `/odom`**，判断视觉信息是否可信。

建议把视觉控制器的指令先送到 `/baseline/cmd_vel`，由防御节点决定是否将其转发到机器人 `/cmd_vel`。指令类型是 **`geometry_msgs/msg/TwistStamped`**。具体话题类型、QoS、启动命令和记录要求见 [防御接入说明](../reproduce-and-defense.md#防御节点接入)。防御算法需要组员实现，当前仓库提供攻击与基线复现基础。

代码入口：[历史帧选择](../../src/tb3_visual_baseline/tb3_visual_baseline/replay_core.py)、[图像中继](../../src/tb3_visual_baseline/tb3_visual_baseline/replay_node.py)、[红色检测](../../src/tb3_visual_baseline/tb3_visual_baseline/vision.py)、[视觉控制](../../src/tb3_visual_baseline/tb3_visual_baseline/node.py)。
