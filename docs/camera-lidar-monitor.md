# Camera LiDAR consistency monitor

Implementation for Issue #9. This integrated node combines the existing HSV visual detector with the trusted LiDAR guard. It is a coarse consistency monitor for the fixed straight route and distinctive centered red box, not a general camera–point-cloud fusion model.

## Decisions and ownership

- Nominal visual stop: red area ratio reaches 0.4, as in the current Camera-only baseline.
- Verification visual cue: red area ratio reaches 0.06. This lower threshold asks whether the known obstacle should already be visible, not whether to brake.
- LiDAR emergency stop: reuse the exact LiDAR-only threshold formula and geometry, without waiting for camera confirmation.
- Persistent discrepancy: three distinct valid pairs disagree in the same direction; latch ANOMALY_STOP.
- Transient discrepancy: record it; continue subject to the normal visual stop and trusted LiDAR guard.
- Missing/stale/unusable required observations: wait before movement; protect motion after starting. Image conversion failure latches SENSOR_INVALID. Backward timestamps latch FAULT.
- Only this integrated node publishes final TwistStamped commands. Do not run the old visual controller or LiDAR controller at the same time. A small shared readiness hook was added to the LiDAR base class; it defaults to true and leaves LiDAR-only behavior unchanged.

An inconsistency is an anomaly, not proof of attack. A visual request to stop is retained even if LiDAR does not see an obstacle. This conservative policy can create extra stops; it does not automatically override visual obstacles using LiDAR.

## Pairing and confirmation

The monitor consumes the oldest available image/scan pair within a 0.12 simulation-second tolerance. Each observation is consumed once. Duplicate timestamps are ignored; skipped frames and repeated timer ticks do not increase confirmation counts. Queues are bounded to thirty observations per sensor. A 0.6 s inter-pair gap restarts evidence accumulation. If required pairing stops, motion is stopped through the shared freshness checks.

Both agreement and ambiguous-distance observations reset the current confirmation count. Opposite disagreement directions do not accumulate together. An established alert remains latched, including if subsequent pairs agree.

The scan is transformed using TF into the robot frame and restricted to the same 0.4 m-wide forward driving corridor as LiDAR-only. Images use the existing central ROI. This is a fixed-scene coarse alignment; the image ROI is not a pixel-level projection of the corridor, and arbitrary off-path red objects or non-red obstacles require further validation.

Fresh output timestamps do not prove fresh image content. This monitor intentionally compares the rewritten-header delayed images against current trusted scans.

## Clean calibration

The clean calibration used the same red obstacle/scene, visual verification ratio 0.06, nominal visual stop ratio 0.4, and 0.3 m/s approach. `calibration_mode=true` records disagreements but disables anomaly-stop requests; visual stopping and LiDAR emergency protection remain active.

The 115 paired observations had a furthest visually positive finite clearance of 2.729203 m and nearest visually negative finite clearance of 2.750881 m. `scripts/calibrate_camera_lidar.py` expands the transition envelope by 0.15 m on each side and rounds outward to two decimals. The resulting YAML band is frozen at 2.57–2.91 m:

- clearance <= 2.57 m: LiDAR expects the red obstacle to be visible;
- clearance >= 2.91 m: LiDAR reports the verification region clear;
- intermediate clearance: ambiguous; no disagreement evidence is accumulated.

The corresponding verification scan range must cover this horizon. An insufficient horizon cannot produce agreement. Calibration is limited to the same obstacle dimensions, placement, detector, camera, ROI and sensor geometry. Recalibrate on clean data when those change. Constructor defaults match the calibrated band. Monitor-specific parameters are startup-only and read-only at runtime; restart the node to change them.

## Build and run

```bash
bash scripts/build_camera_lidar.sh
# Terminal A
bash scripts/start_scene.sh
# Terminal B, clean or configurable delay
bash scripts/run_relay.sh 0.0
# Terminal C, after SCENE_READY and relay readiness
bash scripts/run_camera_lidar.sh drive
```

Use `observe` instead of `drive` for zero-command observation. For a separate project build directory, set TB3_PROJECT_WS; TB3_WS selects the TurtleBot3 dependency workspace. In this local WSL deployment, the helper supports `fusion-drive` and `fusion-observe`. Use the same ROS domain in all terminals. Reset the scene and restart the controller between trials.

The input is `/attack/image_raw`, with SensorData-compatible image subscription QoS. LiDAR and odometry use `/scan` and `/odom`. Final output is `/cmd_vel`; paired diagnostics are JSON String messages on `/defense/consistency`. Per-run logs include base `config.json`, `events.csv`, additional `visual-config.json`, and `pairs.csv`. The latter records timestamps, red ratio, decisions, direction, count and alert state.

## Validation and reproduction

See `results/camera-lidar-2026-11-06/README.md`. Twelve new tests include six real DDS checks. The existing nineteen LiDAR and twenty-nine visual tests pass. Regression logs and raw Gazebo runs are retained.

```bash
# After sourcing ROS and the project install
ROS_DOMAIN_ID=83 python3 -m unittest discover -s src/tb3_camera_lidar_monitor/test -v
ROS_DOMAIN_ID=82 GZ_PARTITION=fusion-check python3 scripts/verify_lidar_gazebo.py \
  --controller fusion --case obstacle --delay 1.0 --ground-truth --output /absolute/new-folder
```

Use `--delay 0.0` for clean, or `--occlusion` with the separately sourced teammate overlay for masking. The verifier checks attack activation before movement. Calibration adds `--calibration-mode --expected-stop VISUAL_STOP` on a clean run. `--empty-world --case clear --goal-distance 4.0 --delay 0.0` checks a full empty route.

This issue demonstrates implementation and development checks, not statistical superiority. Single calibrated obstacle runs and a clean empty-route check cannot establish general false-alarm rate, robustness or the necessity of fusion. Independent teammate review and the subsequent three-configuration evaluation remain required.
