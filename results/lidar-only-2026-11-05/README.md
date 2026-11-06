# LiDAR only development evidence

Date: 5 November 2026. Environment: local WSL Ubuntu 24.04.5, ROS 2 Jazzy, Gazebo Harmonic, pinned TurtleBot3 sources from `configs/turtlebot3.repos`.

This directory records development validation of the new LiDAR-only baseline, not formal attack/defense evaluation. Code and documentation were implemented with Codex assistance. The automated checks were run by the agent; no independent teammate review or manual verification is claimed.

## Checks performed

- `colcon build --symlink-install --base-paths <project>/src --packages-select tb3_visual_baseline tb3_lidar_baseline`: both packages built.
- `ROS_DOMAIN_ID=74 python3 -m unittest discover -s src/tb3_lidar_baseline/test -v`: 16 passed, no skips, including 5 real DDS tests. See `lidar-tests.log`.
- `python3 -m unittest discover -s src/tb3_visual_baseline/test -v`: 29 passed. See `existing-tests.log`.
- `ROS_DOMAIN_ID=75 python3 scripts/verify_lidar_gazebo.py --output <new-folder>`: three sequential checks passed. See `gazebo/summary.json` and raw per-run CSV/config files.
- The same command with `--empty-world`: an actual obstacle-free 0.5 m endpoint check passed, ending at 0.557276 m with zero commanded and measured forward speed. See `empty-world/summary.json`.
- The same command with `--delay 1.0`: all three sequential checks passed with the existing camera relay active. The validator received 348 attacked images and the raw relay CSV confirms REPLAY. See `delay-on/summary.json` and `delay-on/attack/`. The final obstacle-run LiDAR corridor clearance was 1.050348 m and odometry speed was zero.

## Gazebo observations

| Check | Expected and observed state | Final forward displacement from that node's origin | Final odometry speed |
|---|---|---:|---:|
| Clear segment, 0.5 m endpoint | GOAL_REACHED | 0.548276 m | approximately zero |
| Scan stream interrupted, odometry continues | SENSOR_TIMEOUT | 0.107418 m | zero |
| Red obstacle approach | OBSTACLE_STOP | 1.988276 m | zero |

All three ended with zero commanded forward and angular speeds. Obstacle-run final LiDAR corridor clearance was approximately 1.032644 m against the configured stopping threshold of 1.15 m. The threshold is a trigger, not a claim about the final stopping position. The first row shows approximately 0.048 m endpoint overshoot; it is retained rather than rounded into an exact endpoint stop.

## Interpretation limits

- These three checks share one scene and run sequentially without resetting the robot; their origins differ. Displacements must not be compared as absolute world positions.
- The original suite's clear segment ends before the existing distant obstacle. A separate check in a genuinely empty world is recorded under `empty-world/`.
- No camera/visual-decision subscription exists in the LiDAR node. A delay-on development suite has been run, but no matched repeated attack evaluation or Gazebo occlusion-on check is included.
- No contact sensor, video review or independent ground-truth clearance measurement is included. A sensor-derived gap and final zero odometry are not independent proof of no contact.
- Only one sequential development suite is recorded. No attack success rate, false alarm rate or statistically generalizable task completion rate is reported.
- Raw per-run configuration and logs are preserved. Simulation shutdown messages at the end of `scene.log` come from deliberate SIGINT cleanup.

Reproduce using `docs/lidar-only.md`. Complete an independent teammate review and matched repeated experiments before treating this as final project evidence.
