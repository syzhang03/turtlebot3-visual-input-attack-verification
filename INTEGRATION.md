# Camera Attack integration

Base reviewed: main `d5b8bd6` (17 November 2026). This branch non-destructively
merges that main; all teammate branches and historical results remain unchanged.

## Interface

`/camera/image_raw` → `camera_occlusion` → `/camera/attacked_image` (legacy default).
Set `output_topic:=/attack/image_raw` to replace Delayed Frame Replay.
Message type is `sensor_msgs/msg/Image`. Input defaults to Best Effort / Keep Last /
5, output to Reliable / Keep Last / 1, both Volatile. The actual main Monitor uses
SensorData-compatible image subscription QoS; Reliable also serves Reliable consumers.
Configure `{input,output}_{reliability,history,depth}` at startup. History accepts
`keep_last` or `keep_all`; depth is relevant to Keep Last. Avoid Keep All in long runs.

Clean frames pass through byte-for-byte. Attacks preserve the input Header,
timestamp and frame_id. Legacy output is BGR8. `preserve_encoding:=true` supports
BGR8, RGB8, BGRA8, RGBA8 and mono8 without a color conversion; alpha is preserved.
Color shift uses explicitly named B/G/R offsets and rejects mono8. Unsupported
formats/processing failures are dropped and logged, not relabeled or restamped.

**Stop delay_replay and all other camera relays before starting this node.**
The node suppresses its own output if DDS reports another publisher on its resolved
output topic. Discovery is asynchronous: this is a guard, not distributed arbitration.
Use one launch owner and verify publisher count before enabling any controller.
The attack node never publishes `/cmd_vel`. Only one selected controller may own it.

## Ubuntu setup and integration check

The Windows SSH config has no entry for `meiqi@dase7508-vm`; no address or key was
guessed. On the VM console, confirm `hostname` and `whoami`, then:

```bash
cd /home/meiqi/turtlebot3-visual-input-attack-verification
git status --short                  # preserve any VM-local edits first
git fetch origin
git switch hou_meiqi_camera_attack
git pull --ff-only origin hou_meiqi_camera_attack
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
colcon build --symlink-install --packages-up-to tb3_camera_attack tb3_camera_lidar_monitor
source install/setup.bash
colcon test --packages-select tb3_camera_attack
colcon test-result --verbose
python3 -m pytest --import-mode=importlib src/tb3_camera_attack/test -q
ros2 pkg executables tb3_camera_attack
# Isolated real DDS test: raw Image -> attack -> actual CameraLidarMonitor.
# Monitor disabled; its commands are remapped to /camera_attack_test/cmd.
ROS_DOMAIN_ID=84 python3 -m unittest discover \
  -s src/tb3_camera_attack/test -p test_monitor_integration.py -v
```

This test requires ROS/cv_bridge and the merged monitor package. A skipped test is
**Not Tested**, not integration success. It checks image delivery to a Reliable
subscriber plus actual monitor image acceptance and camera/scan pairing.

## Live scene without changing the defense

In three sourced terminals with the same `ROS_DOMAIN_ID=33`:

```bash
# A: latest main's unchanged scene
bash scripts/start_scene.sh
# B: REPLACES run_relay.sh, never run both
ros2 run tb3_camera_attack camera_occlusion --ros-args \
  --params-file src/tb3_camera_attack/config/integration.yaml
# C: zero-motion observer first
bash scripts/run_camera_lidar.sh observe
```

Before any drive trial, inspect `ros2 topic info /attack/image_raw --verbose`:
exactly one publisher, Image type, and compatible QoS. Also run:

```bash
ros2 topic echo /attack/image_raw --once --field header --qos-reliability reliable
ros2 topic echo /defense/consistency --once
ros2 param set /camera_occlusion attack_enabled true
ros2 param set /camera_occlusion attack_mode gaussian_noise
ros2 param set /camera_occlusion noise_sigma 15.0
ros2 param set /camera_occlusion attack_enabled false
```

Seeing a topic name alone is insufficient: require received images and diagnostics
or `pairs.csv` rows containing new image timestamps. In observe mode the monitor
does not drive. Stop it before starting a drive controller. Restart scene, relay
and controller between runs. Latest main uses speed 1.0, visual stop 0.4,
verification 0.06, ROI top 0.25, clearance 0.35, band 2.57–2.91 and 3 confirmations.
Do not retune these based on attack outcomes.

The existing evaluator already supports this package's default occlusion:

```bash
ROS_DOMAIN_ID=85 GZ_PARTITION=meiqi-check python3 scripts/verify_lidar_gazebo.py \
  --controller fusion --case obstacle --occlusion --ground-truth \
  --output "$HOME/meiqi-camera-results/occlusion-$(date +%Y%m%dT%H%M%S)"
```

It starts its own relay and controller: stop manual relays/controllers first.
`run_issue13.py` and `run_issue13.py --lidar-only` retain the frozen original
protocol; use them only for its supported clean/replay/default-mask conditions.
They do not support an arbitrary perturbation-mode CLI. For new intensities use
the camera sweep below and retain perception results separately; a future dynamic
evaluator adapter is needed for automated noise/brightness safety campaigns.

## Scope and status

The joint camera–LiDAR branch is `163589a`; its LiDAR attack package and frozen
joint-attack profile are not in main `d5b8bd6`. They were inspected, not merged.
To combine later, merge that branch into a reviewed integration branch and retain
the same Image topic; no defense algorithm change is required by this camera module.
This work does not assert that joint-attack branch has been integrated or tested.

ROS build/DDS/Gazebo, live red_area, stopping, displacement, clearance, timeout
and anomaly results: **Not Tested on this Windows host**. Existing results are
teammates' historical evidence, not new measurements. `red_area=0` is not collision.
To enable remote tests, provide an authorized SSH host/address and existing login
method, or run the commands from the VMware console. No VM network changes are needed.
