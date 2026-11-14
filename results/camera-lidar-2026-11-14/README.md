# Camera and LiDAR input attacks: 14 November 2026

This update provides a working two-sensor attack testbed, four matched development trials, and a starting point for teammates implementing defenses. The robot uses the existing houses, road, white stop line and fixed red obstacle. All four retained trials used the same camera–LiDAR controller at **1.0 m/s** with a **0.4 visual stop threshold** and a **0.788 m fixed LiDAR trigger clearance**.

The single-sensor attacks were followed by stops using the other sensor. The joint attack delayed the stop: final logged forward displacement increased from **2.9653 m to 3.7083 m**, a difference of **0.7430 m**. A later independent Gazebo geometry check found a **current clearance lower bound of 0.064162 m (6.42 cm)** while the robot was stationary. **A collision was not established.** The check cannot reconstruct contact earlier in the approach.

This is an extension of the original camera-only threat model. Here LiDAR may also be compromised; it must not be treated as trusted in the `lidar` and `joint` conditions. The repository's original root README is retained. These four conditions are not the historical four conditions in Issue #13, and the results must not be pooled with that evaluation or the October camera-only trials.

## 1. Environment and completed work

| Item | Configuration |
|---|---|
| Host/guest | VMware, Ubuntu 24.04; 4 virtual CPUs and approximately 8 GiB RAM |
| Robot stack | ROS 2 Jazzy, Gazebo Harmonic, TurtleBot3 Waffle Pi; dependency revisions in [turtlebot3.repos](../../configs/turtlebot3.repos) |
| Scene | [intersection-white-line.sdf](../../worlds/intersection-white-line.sdf): houses, road and white line; red box centered at `(4.0, 0.0, 0.25)` m, size `0.3 × 0.5 × 0.5` m |
| Start and motion | Robot starts at `(0, 0)` facing +X; forward approach with heading correction. The white line is a visual reference, not a detected control target. |
| Camera | 640 × 480 RGB images in the observed stream; largest red contour area divided by the central ROI area, not the percentage of red pixels in the full frame |
| Infrastructure | One-command scene startup; bounded camera replay; configurable LiDAR bias relay; common experiment manifest and per-node logs; transport diagnostics; stop-point calibration and post-stop geometry check |

The camera path was checked against Gazebo's native stream. After selecting reliable camera relay input/output and a reliable controller image subscription, a 60-second check observed 546 frames on each ROS image topic, maximum receive gaps of 0.202/0.211 wall seconds, and a maximum image-header gap of 0.034 simulation seconds. The native observer saw 563 frames; unequal observation start times mean total counts alone are not a loss measurement. Earlier timeout/debug runs are excluded from the four retained trials below. Reliable QoS improves this tested setup but is not a real-time guarantee.

## 2. Camera attack: delayed historical-frame replay

```text
/camera/image_raw -> delay_replay -> /attack/image_raw -> controller/defense
```

The attacker controls the image stream between capture and perception. For an incoming image timestamp `t` and requested delay `d`, the relay:

1. Stores incoming images in a bounded chronological buffer.
2. Selects the newest stored image whose original timestamp is at or before `t - d`.
3. Publishes that historical image payload with the **current incoming message header**.
4. Logs both the current input timestamp and the selected historical timestamp, so `actual_delay_s = t - selected_source_timestamp` is auditable.

The delay is measured in **simulation time**, not wall-clock time. A 1.0-second request produced 1.022–1.024 seconds of actual content delay in the retained attack trials because selection is quantized by image sampling. The controller sees an older, apparently more distant obstacle, while an ordinary header freshness check can still see a recent timestamp. Timestamp freshness therefore does not establish content freshness.

With `delay=0`, the relay forwards the current image. Before enough history is available, the relay reports `BUFFER_NOT_READY`; only start the moving attack trial after `REPLAY`. Duplicate timestamps, buffer capacity, image-format changes and time resets are handled explicitly. Fallback states are recorded and must not be mistaken for active replay. This is a simulated compromised image interface, not a method of initially compromising ROS or a physical camera.

Implementation: [replay_core.py](../../src/tb3_visual_baseline/tb3_visual_baseline/replay_core.py), [replay_node.py](../../src/tb3_visual_baseline/tb3_visual_baseline/replay_node.py).

## 3. LiDAR attack: positive range bias in a forward sector

```text
/scan -> lidar_bias -> /attack/scan -> controller/defense
```

For each valid finite range `r_i`, its angle is computed from `angle_min + i × angle_increment`. If that angle is within **±30 degrees of the scan frame's +X direction**, the active attack publishes:

```text
r'_i = min(range_max, r_i + 1.0 m)
```

Ranges outside the sector are unchanged. NaN, infinity and out-of-range readings are preserved rather than turned into free space. The relay retains the scan header, angles, timing and intensities. With zero bias, it passes the original scan through. A nonzero bias can be clipped at `range_max`, so not every modified range changes by a full metre.

The intended effect is to make the obstacle appear farther away and delay a LiDAR-based stop. The bias is radial; it is **not** an exact one-metre change in vehicle-to-obstacle clearance. TF transformation, corridor selection, unaffected rays and the camera can still trigger stopping. Check both `BIAS_ACTIVE` and nonzero `changed_rays`; an active schedule alone does not prove that any range changed.

Implementation: [core.py](../../src/tb3_lidar_attack/tb3_lidar_attack/core.py), [node.py](../../src/tb3_lidar_attack/tb3_lidar_attack/node.py). Neither attack relay publishes velocity commands or changes the simulated obstacle.

## 4. Common controller and calibration

| Parameter | Value used in all four trials |
|---|---:|
| Commanded forward speed | 1.0 m/s |
| Visual stop / visual consistency threshold | 0.4 / 0.4 |
| LiDAR stop mode / threshold | Fixed / 0.788 m |
| Consistency near / far boundaries | 0.71 / 0.87 m |
| Camera–LiDAR pairing tolerance | 0.06 simulation seconds |
| Consecutive disagreements required | 3 |
| Pair timeout | 0.6 simulation seconds |
| Sensor receive timeout / maximum header age | 2.0 wall seconds / 0.5 simulation seconds |
| Corridor half-width / front extent | 0.20 / 0.15 m |
| Endpoint stop | Disabled |
| Image relay input/output and controller image subscription | Reliable, keep-last depth 1 |

The fixed distance was calibrated from a clean 0.3 m/s approach: red ratios 0.378668 and 0.413947 bracketed 0.4 at corridor clearances 0.808748 and 0.775025 m. Linear interpolation gave 0.788357 m, rounded to 0.788 m. The consistency band includes a 0.06 m allowance for motion during the formal 1.0 m/s pairing tolerance. It is not a statistical confidence interval or a universal calibration.

**Corridor clearance** is computed by transforming scan points into `base_footprint`, selecting the forward corridor and subtracting `front_extent`. It differs from raw radial range, odometry displacement and actual collision-geometry separation. Fixed mode does not add the dynamic reaction/braking terms to 0.788 m.

The controller can latch `VISUAL_STOP` from the red-area threshold, `OBSTACLE_STOP` from LiDAR clearance, or `ANOMALY_STOP` from three consecutive consistency disagreements. Independent sensor stopping can occur before consistency confirmation. An ordinary obstacle stop is not evidence that the consistency monitor detected an attack. A latched terminal stop keeps publishing zero commands; a quiet console after that transition is expected.

The [frozen reference profile](../../configs/aligned_stop_20261114.yaml) is transcribed from the reported calibration/configuration. [Calibration instructions](../../docs/aligned-stop-calibration.md) explain how to obtain a new profile if the scene, robot, ROI or geometry changes.

## 5. Four retained trials and results

Each row is **one development trial**, not a repeated statistical evaluation. All four summaries reported `issues: []`; this is a configuration/data audit, not a collision-success criterion. Results below are transcribed from the operator's terminal summaries and observations. The original complete VM CSV/JSON directories have **not yet been copied into this evidence folder**; [reported-results.json](reported-results.json) records provenance and the original run identifiers. No raw sensor records have been reconstructed or invented.

| Condition | Requested camera delay | Requested LiDAR bias | First stop state | Forward displacement at first zero command | Forward displacement at first logged zero odometry speed | Contact evidence |
|---|---:|---:|---|---:|---:|---|
| `clean` | 0 s | 0 m | `VISUAL_STOP` | 2.9515 m | 2.9653 m | Operator reported no contact and supplied a screenshot in the working session |
| `camera` | 1 s | 0 m | `OBSTACLE_STOP` | 3.0025 m | 3.0263 m | Operator confirmed no contact and a complete stop |
| `lidar` | 0 s | +1 m | `VISUAL_STOP` | 2.9525 m | 2.9733 m | Operator confirmed no contact and a complete stop |
| `joint` | 1 s | +1 m | `OBSTACLE_STOP` | 3.6815 m | 3.7083 m | Later native geometry check: currently separated by at least 0.064162 m; earlier contact remains unknown |

| Condition | Actual accepted camera content delay (min / mean / max) | Matched accepted camera frames | LiDAR relay states during motion | Cumulative changed ray samples |
|---|---|---:|---|---:|
| `clean` | 0 / 0 / 0 s | 107 | 35 `PASSTHROUGH` | 0 |
| `camera` | 1.022 / 1.023 / 1.024 s | 106 | 35 `PASSTHROUGH` | 0 |
| `lidar` | 0 / 0 / 0 s | 105 | 34 `BIAS_ACTIVE` | 444 |
| `joint` | 1.022 / 1.022992 / 1.024 s | 127 | 42 `BIAS_ACTIVE` | 812 |

Changed ray samples are summed across scans, not counts of distinct laser beams. LiDAR counts describe relay output during motion; they do not independently prove delivery of every scan to the controller. All four excerpts show `cmd_v` falling from 1.0 to 0.0, followed by `odom_v` reaching 0.0: the controller commanded a stop, rather than merely continuing to drive against a block.

The joint trial's reported corridor clearance dropped from 1.1049 to 0.0210 m at stopping. Those are **attacked sensor measurements**, not independent clearance measurements. Unmodified sector-edge rays are one possible explanation, but the current summaries do not establish which ray triggered the stop.

The subsequent geometry check observed 177 native-pose samples over five wall seconds and reported stationary, separated collision geometry. Its 6.42 cm value is a conservative **lower bound during that check**, conditional on the installed collision model matching the live scene. It is not an exact gap or the minimum clearance over the whole approach. With collision as the success criterion, the joint trial is **not a demonstrated collision success**. No 1.2 m/s trial was completed for this update.

Original VM run identifiers (UTC):

```text
clean:  20261114T114534.386523Z / controller/20261114T114609.164320Z
camera: 20261114T115158.435562Z / controller/20261114T115219.316460Z
lidar:  20261114T115634.182537Z / controller/20261114T115651.666679Z
joint:  20261114T120312.856458Z / controller/20261114T120329.817970Z
post-stop geometry: clearance-checks/20261114T121443.163765Z
```

## 6. Reproduce on another Ubuntu machine

### Install and build once

Prerequisites: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and a working Waffle Pi dependency workspace at `~/turtlebot3_ws`. For a new machine, follow the dependency installation section of [the existing setup guide](../../docs/reproduce-and-defense.md), using the branch and directory below instead of its old camera-only branch. The pinned dependency file is [configs/turtlebot3.repos](../../configs/turtlebot3.repos).

```bash
git clone --branch wuahonan-attack-2-camera-lidar \
  https://github.com/syzhang03/turtlebot3-visual-input-attack-verification.git \
  "$HOME/camera-lidar-attack"
cd "$HOME/camera-lidar-attack"
source /opt/ros/jazzy/setup.bash
source "$HOME/turtlebot3_ws/install/setup.bash"
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
bash scripts/build_joint_attack.sh
source install/setup.bash

# Fresh clone only: activate the profile used in the reported trials.
cp configs/aligned_stop_20261114.yaml configs/aligned_stop.yaml

# No driving or Gazebo required for these tests; domain 34 separates DDS tests.
ROS_DOMAIN_ID=34 python3 -m unittest discover -s src/tb3_lidar_attack/test -v
ROS_DOMAIN_ID=34 python3 -m unittest discover -s src/tb3_visual_baseline/test -v
ROS_DOMAIN_ID=34 python3 -m unittest discover -s src/tb3_lidar_baseline/test -v
ROS_DOMAIN_ID=34 python3 -m unittest discover -s src/tb3_camera_lidar_monitor/test -v
python3 -m unittest discover -s tests -v
```

Build on Ubuntu's local filesystem, not the VMware shared mount. Do not overwrite a teammate's active calibration without saving it. If `TB3_WS` differs, set it consistently in all terminals. Existing Windows-share copies should not be used as a substitute for fetching this published source revision.

### Run each condition with a fresh scene

Before every trial, stop the old controller, relays and scene with Ctrl+C, in that order. Close teleop or other velocity publishers. Keep the same GUI/rendering workload and ROS domain across conditions; use different domains for independent experiments on a shared network.

**Terminal A — start the scene and wait for `SCENE_READY`:**

```bash
ROS_DOMAIN_ID=33 bash "$HOME/camera-lidar-attack/scripts/start_scene.sh"
```

**Terminal B — choose exactly ONE command per fresh trial:**

```bash
# No attack: both relays pass through.
ROS_DOMAIN_ID=33 bash "$HOME/camera-lidar-attack/scripts/run_joint_attack.sh" clean

# Camera only: 1.0 simulation-second historical content delay.
ROS_DOMAIN_ID=33 bash "$HOME/camera-lidar-attack/scripts/run_joint_attack.sh" camera 1.0

# LiDAR only: camera delay 0, range bias +1.0 m, sector half-width 30 degrees.
ROS_DOMAIN_ID=33 bash "$HOME/camera-lidar-attack/scripts/run_joint_attack.sh" lidar 0.0 1.0 30.0

# Joint: apply both manipulations.
ROS_DOMAIN_ID=33 bash "$HOME/camera-lidar-attack/scripts/run_joint_attack.sh" joint 1.0 1.0 30.0
```

Wait for `ZERO_DELAY` or `REPLAY` on the camera relay, and `PASSTHROUGH` or `BIAS_ACTIVE` on the LiDAR relay as appropriate. For active LiDAR bias, check that modified rays are logged. Attack windows run for 600 simulation seconds from each stream's first input; restart a trial if its window expires. `1.0` is a number; `1.0.0` is not valid input.

**Terminal C — same controller command for every condition:**

```bash
ROS_DOMAIN_ID=33 bash "$HOME/camera-lidar-attack/scripts/run_joint_controller.sh" drive
```

Check startup for `Controller profile: aligned`, `mode=fixed`, `threshold=0.7880m`, `speed=1.0` and reliable `/attack/image_raw`. The active profile is copied into the run directory. First validate `clean`, then run the other three conditions. Preserve all runs, including timeouts and unsuccessful attacks; repeat each condition for a formal evaluation.

### Save and inspect evidence

After stopping, allow several seconds of logging. Preserve the current scene if measuring the final gap. In another terminal:

```bash
run_dir="$(cat "$HOME/joint-attack-results/latest-run.txt")"
python3 "$HOME/camera-lidar-attack/scripts/summarize_joint_trial.py" "$run_dir"
bash "$HOME/camera-lidar-attack/scripts/check_stopped_clearance.sh" "$run_dir"
```

The geometry helper needs Gazebo Harmonic's `gz.transport13` and `gz.msgs10` Python bindings. It reads native model/link poses and collision primitives, publishes no commands, and stores its report under `~/clearance-checks`. `CURRENTLY_SEPARATED` applies only to the check window; overlapping bounds do not prove contact. A reset destroys the original stopped pose. For whole-trial collision scoring, teammates must add an independent contact recorder before motion; this update does **not** include a completed full-run contact-recording system.

Preserve `manifest.json`, `controller-profile.yaml`, both relays' configuration and CSV files, and controller `config.json`, `visual-config.json`, `events.csv`, `pairs.csv` and `image-acceptance.csv`, together with screenshots and independent geometry/contact evidence. Directory timestamps are UTC; they can differ from local display time.

For intermittent image input, run the passive diagnostic before driving and then close it to keep experimental load consistent:

```bash
ROS_DOMAIN_ID=33 bash "$HOME/camera-lidar-attack/scripts/check_camera_streams.sh" \
  --seconds 60 --gazebo --raw-reliability reliable
```

Investigate `SENSOR_TIMEOUT` using [diagnose_joint_timeout.py](../../scripts/diagnose_joint_timeout.py). Do not report infrastructure failures as demonstrated collision attacks.

## 7. How teammates can build and evaluate defenses

Keep the scene and relays. Replace Terminal C with the teammate's controller/defense, or modify `tb3_camera_lidar_monitor` on a separate branch. The supplied monitor is a reproducible comparator, not a claim of comprehensive protection.

| Interface | Type / policy | Use |
|---|---|---|
| `/attack/image_raw` | `sensor_msgs/msg/Image`; reliable, keep-last 1 | Camera input to the defense in all four conditions |
| `/attack/scan` | `sensor_msgs/msg/LaserScan`; reliable relay publisher; current monitor uses SensorData subscriber QoS | LiDAR input to the defense in all four conditions |
| `/odom` | `nav_msgs/msg/Odometry`; SensorData QoS | Motion information, unchanged by these relays |
| `/tf`, `/tf_static` | Standard TF interfaces | Transform scan geometry to `base_footprint` |
| `/clock` | `rosgraph_msgs/msg/Clock` | Set `use_sim_time=true` |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Exactly one controller publishes the final velocity command |
| `/defense/consistency` | `std_msgs/msg/String` containing JSON | Existing monitor's diagnostic output; not an input that automatically stops an unrelated controller |

Practical integration steps:

1. Create a defense branch from this attack branch and build the new package in the same sourced environment. Run `clean` successfully before changing defense logic.
2. Subscribe to **both attacked topics**, even for `clean`. Reading `/scan` in the `lidar` or `joint` group would bypass the attack. Raw topics, relay provenance and Gazebo ground truth are reserved for evaluation unless explicitly added to the defense's trust model.
3. Set the same 1.0 m/s speed, geometry, simulation clock and frozen thresholds for the first comparison. Use the two input streams to implement the new check and connect its decision to the sole final command publisher. Do not run the supplied drive controller concurrently with another drive controller.
4. Save the defense version, effective parameters, input acceptance times, alarm reason/time, outgoing commands and actual motion. The supplied summary expects the supplied controller's log schema; a different controller needs a compatible exporter or its own evaluator.
5. Run `clean`, `camera`, `lidar` and `joint` from fresh scenes with the same attack settings. Measure false stops, detection latency, stopping behavior, full-run independent contact and minimum clearance. Report ordinary sensor stops separately from anomaly alarms.

Historical pixels with a current header can defeat header-only freshness checks. In the joint condition, mutually plausible observations may still both be wrong: cross-sensor agreement alone does not establish truth. These are properties to test, not reasons to assume every joint attack will collide.

## 8. Evidence and validation limits

The four VM runs and the post-stop check are operator-provided evidence. This publication includes their explicitly labeled summary, the scripts used, the recorded parameter profile, and tests. Full raw VM records and a full-run contact trace are pending. Local test results and skipped ROS checks are recorded in [validation.json](validation.json). Windows unit tests do not substitute for a ROS build or live Gazebo trial on a teammate's machine.
