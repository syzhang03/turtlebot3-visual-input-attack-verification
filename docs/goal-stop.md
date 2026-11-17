# 关闭当前实验的终点停车

用户要求消除 4 m 终点停车对延迟重放实验的干扰。当前场景配置 `baseline_white_line.yaml` 设置：

```yaml
speed: 1.0
red_area_threshold: 0.4
enable_goal_stop: false
goal_distance: 4.0
sensor_timeout: 2.0
max_trial_seconds: 120.0
```

`enable_goal_stop=false` 时 `goal_distance` 不参与停止判断，超过 4 m 不会产生 GOAL_REACHED。
视觉触发停车、传感器超时、无效数据/时钟故障、120 秒仿真运动时限和 Ctrl+C 的零速度发送均保留。
节点参数默认仍为 true，兼容独立使用的旧基线；当前直连和中继启动脚本使用上述实验配置，均关闭终点停车。

## 一条命令更新 Ubuntu

先停止旧控制器和中继，再执行：

```bash
bash /mnt/hgfs/VM_share/delay-replay-2026-10-30/scripts/update_goal_stop.sh
```

更新器同步状态机、节点、测试与配置，备份旧源码，执行构建和测试。看到 UPDATE_COMPLETE 后重启场景。
控制器启动时必须显示：

```text
Control: speed=1.0; red_threshold=0.4; goal_stop=False; sensor_timeout=2.0
```

启动命令不变：

```bash
# 终端 A，先等待 SCENE_READY
ROS_DOMAIN_ID=33 bash ~/delay-replay-2026-10-30/scripts/start_scene.sh

# 终端 B，按实验条件填写 0.0、0.5、1.0；等待 ZERO_DELAY 或 REPLAY
ROS_DOMAIN_ID=33 bash ~/delay-replay-2026-10-30/scripts/run_relay.sh 0.0

# 终端 C，行驶
ROS_DOMAIN_ID=33 bash ~/delay-replay-2026-10-30/scripts/run_relay_baseline.sh
```

配置在启动时读取，必须重启控制器才能生效；运行中的旧节点不会自动变化。
使用其他配置或手工命令时，应明确增加 `-p enable_goal_stop:=false`。不要把终点距离设为 0 来替代开关。

## 首次停车原因

旧版图像回调会把已有的 GOAL_REACHED 等停止状态覆盖成 OBSTACLE_STOP。新版锁定第一次停车原因，后续图像判断仍会写入 `visual_blocked` 和 `red_area_ratio`，不会把先到终点停车的记录改成视觉停车。
后续发生的异常依然可打印错误，但不会覆盖第一次停车原因。
每次运行的 config.json 增加 enable_goal_stop 和 control_revision，便于区分代码版本。

## 对照与证据

后续直连、0 秒中继和全部延迟组使用同一新版配置。正式比较应重新采集这套条件下的数据。
尤其是 20261030T150208.755759Z：旧日志先在 4.016 m 触发 GOAL_REACHED，随后才显示视觉停止，须保留为旧条件结果，不能改写成新配置下的成功视觉停车。
若发生接触或未停车，由操作者停止控制器并如实记录；没有终点停车不代表自动具备碰撞检测。

控制与缓冲测试覆盖禁用终点后继续运行、视觉停车与超时保留、首次停止原因锁定。后续 Ubuntu 试验 20261030T153945.096132Z 的原始配置确认 enable_goal_stop=false，首次停止为 OBSTACLE_STOP；详见 [证据目录](../results/delay-replay-2026-10-30/README.md)。本次本地测试范围与 ROS 环境限制另见其中的验证记录。
