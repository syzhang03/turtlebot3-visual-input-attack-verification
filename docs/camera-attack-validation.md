# Camera attack review and validation record

Date: 2026-11-17. Branch: hou_meiqi_camera_attack. Pre-upgrade head: fa0706a.
Reviewed remote main: d5b8bd6. Non-destructive merge: 0321464, no conflicts.
Local main and all teammates' development branches were left unchanged.

## Remote branch audit

| Branch | Reviewed head | Relevant interface |
|---|---|---|
| wu-haonan-visual-baseline | aeaba06 | Historical HSV baseline |
| wuahonan-attack-1-delayed-frame-replay | 1410e84 | Image replay and current headers |
| wuahonan-attack-2-camera-lidar | 163589a | Joint attack, separate LiDAR package, not merged |
| defense/camera-lidar-monitor | 748309c | Image + scan pairing; /attack/image_raw |
| defense/lidar-only | 9ca52de | Trusted scan/odom guard |
| hou_meiqi_camera_attack | fa0706a before merge | Legacy occlusion, /camera/attacked_image |

Read main README/UPDATE_NOTES, monitor and Issue 13/15 docs, relay transport,
actual replay and Monitor code/config, verifier command construction and the
white-line scene. The joint-attack guide was read from its own branch; that
branch's additional LiDAR module/profile are absent from main. No joint-attack
merge or experiment is claimed. main already includes default-mask evaluations,
so this upgrade does not repeat or reinterpret those formal results.

## Executed on Windows

Python 3.14 test environment outside the repository, numpy 2.5.5,
opencv-python-headless 5.0.0.93, pytest 9.2.0, PyYAML 6.0.3.

```powershell
$env:PYTHONPATH='src/tb3_camera_attack;src/tb3_visual_baseline;src/tb3_camera_lidar_monitor;src/tb3_lidar_baseline'
python -m pytest --import-mode=importlib src/tb3_camera_attack/test src/tb3_visual_baseline/test src/tb3_camera_lidar_monitor/test src/tb3_lidar_baseline/test -q
```

Actual result: **83 passed, 22 skipped, 36 subtests passed**. Skips require ROS,
CvBridge or real DDS. Initial combined collection failed because teammate tests
share module filenames; importlib collection fixes the invocation without
editing their tests. `git diff --check` passed. Historical results have no changes
relative to origin/main. No ROS mocks are presented as real ROS execution.

Coverage includes legacy occlusion and detector regression, pure perturbation
math, clipping, reproducible/different seeds, ROI validation, schedule boundaries,
channel order/alpha, clean passthrough, header copying with an explicit bridge
double, preset validation and the Image-only publisher contract. Retained and
new ROS tests cover runtime changes, invalid atomic updates, real RGB encoding,
Header and timestamp preservation, Reliable delivery and actual Monitor input.
Those ROS tests were **not executed** on this host.

## Not Tested and remaining work

- No SSH host/address/key for meiqi@dase7508-vm exists in the inspected config.
  The existing unrelated SSH host was not contacted. No VM network, firewall,
  SSH service or shared-folder settings were changed.
- VM identity, ROS environment, colcon build/test, real image transport and
  Gazebo: Not Tested. Commands are in INTEGRATION.md. The Bash recorder was
  reviewed but not run on this Windows host.
- No new red_area, stop time, displacement, clearance, anomaly, timeout or
  collision measurements exist. No attack success rate is claimed.
- Arbitrary perturbation safety campaigns still need an evaluator adapter;
  the unchanged main verifier supports the default mask, not general intensity
  settings. Use the provided one-trial bag recorder to collect new inputs,
  retaining invalid and unsuccessful trials. Never overwrite formal results.

## Change map

- perturbation.py: validated attack settings, timing and pure pixel operations.
- node.py / relay.py: runtime settings, RNG, image adaptation, QoS and collision
  guard for topic publishers, source stamps, optional per-frame logs.
- camera_occlusion.launch.py / setup.py / package.xml: topic arguments, preset
  installation and YAML test dependency.
- config/*.yaml: backward-compatible original plus initial intensity presets.
- test_perturbation.py / test_configs.py: portable semantic and contract tests.
- test_ros_relay.py / test_monitor_integration.py: real ROS tests (skipped here).
- run_camera_attack_trial.sh: fresh-folder bag capture and provenance, no control.
- INTEGRATION.md / camera-attack-upgrade.md: deployment, team handoff and usage.
