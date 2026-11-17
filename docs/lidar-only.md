# LiDAR only baseline

This baseline provides the independent LiDAR comparator requested in the instructor's feedback. It controls forward navigation using `/scan`, `/odom`, TF and the clock. It does not subscribe to camera images, visual detections, or the visual controller's commands. It is a controlled, planar, straight-route prototype, not a general navigation planner or a certified safety controller.

## Current behavior

The node transforms each scan into `base_footprint` using TF at the scan timestamp. It selects points with `x >= 0` and `abs(y) <= corridor_half_width`, computes longitudinal clearance `max(0, x - front_extent)`, and stops when the closest selected point reaches the stopping threshold:

```text
threshold = min_clearance + v * reaction_time + v² / (2 * braking_deceleration)
v = max(configured speed, nonnegative measured forward speed)
```

Default speed is 1.0 m/s to match the current attack configuration. Default threshold at this speed is 1.15 m. `front_extent=0.15 m` and `corridor_half_width=0.20 m` define a conservative envelope for this Waffle Pi experiment. These settings, the reaction allowance and the assumed 1.0 m/s² braking deceleration are development parameters; they are not independently calibrated braking guarantees. Freeze calibrated parameters before formal evaluation.

Unlike a minimum-over-all-rays detector, an object outside the driving corridor does not directly cause an obstacle stop. An obstacle within the stopping threshold causes immediate zero commands without consecutive-frame confirmation. No slowing or obstacle bypassing is implemented.

The first terminal state is latched until process restart:

| State | Meaning |
|---|---|
| WAITING | Initial valid, fresh inputs and clock are not ready |
| DISABLED | Observation mode; zero commands only |
| RUNNING | Forward motion with bounded initial-heading correction |
| OBSTACLE_STOP | Path obstacle reached the stopping threshold |
| GOAL_REACHED | Forward odometry displacement reached the endpoint |
| SENSOR_TIMEOUT | Scan or odometry reception/stamps became stale after motion started |
| SENSOR_INVALID | An unusable scan/transform arrived after motion started |
| FAULT | Backward clock/stamp, invalid odometry or unsupported nonplanar pose |
| TRIAL_TIMEOUT | Simulation-time trial limit exceeded |
| COMMAND_CONFLICT | Another final command publisher was discovered |

Publisher discovery is asynchronous: the conflict check helps catch integration mistakes but is not a security boundary or a replacement for starting exactly one controller. Even observation mode publishes zeros, so do not run it beside another controller.

## Scan and timing assumptions

- Require forward semicircle coverage, positive angular increments no greater than 5 degrees, and sufficient advertised scan range to cover the stopping region.
- NaN, negative infinity and finite out-of-range forward readings invalidate the scan. Partial invalid forward scans are not treated as clear.
- Positive infinity is explicitly accepted as a no-return ray under the Gazebo driver convention (`allow_positive_infinity=true`). Set it false for a source where this convention is not valid. This assumption is not proof that every physical object is observable.
- Missing TF or unsupported roll/pitch makes the scan unusable. The scan frame is not assumed to be the robot origin.
- Scan and odometry timestamps must advance. Repeated stamps do not refresh reception watchdogs; backward stamps latch a fault.
- Both steady reception age (default 2 wall seconds) and message stamp age (default 0.5 simulation seconds) are checked. A steady timer continues sending zero commands if simulation time pauses.
- Before motion starts, missing/unusable inputs keep the node waiting; after motion begins they trigger a latched stop.

LiDAR remains trusted under the project threat model. This baseline cannot detect obstacles outside its scan plane or resolve LiDAR compromise. Camera attacks cannot affect its decision inputs; runtime overhead from other processes can still affect timing.

## Build and run

In Ubuntu, source ROS and the pinned TurtleBot3 dependency workspace. For the standard in-repository build:

```bash
bash scripts/build_lidar_baseline.sh
bash scripts/run_lidar_baseline.sh observe
```

Start the existing scene in a separate terminal and wait for `SCENE_READY`:

```bash
ROS_DOMAIN_ID=33 bash scripts/start_scene.sh
```

Stop observation mode before enabling motion:

```bash
ROS_DOMAIN_ID=33 bash scripts/run_lidar_baseline.sh drive
```

No camera relay or visual controller is needed for LiDAR-only. To override a parameter, append ROS arguments, for example `-p goal_distance:=1.0`. The default endpoint is enabled at 4 m; to match the existing obstacle attack trials that disable endpoint stopping, append `-p enable_goal_stop:=false`. Use identical endpoint policy across formal comparator configurations.

For a build workspace outside the repository, set `TB3_PROJECT_WS` to its absolute Linux path. `TB3_WS` selects the TurtleBot3 dependency workspace. Logs are written to `~/lidar-results/<UTC-run-id>/config.json` and `events.csv`.

## Tests

```bash
source /opt/ros/jazzy/setup.bash
source "${TB3_PROJECT_WS:-$PWD}/install/setup.bash"
python3 -m unittest discover -s src/tb3_lidar_baseline/test -v
```

There are 11 geometry/state tests and 5 real ROS DDS tests. ROS tests check goal stopping, latched obstacle stopping, interrupted scans with ongoing odometry, repeated old scan rejection, and invalid scan handling. They use synthetic messages on `/lidar_test/*`, not robot actuators. Run them in an isolated domain, e.g. `ROS_DOMAIN_ID=74`. They explicitly skip if ROS imports are unavailable; skipped tests are not passes.

For the bounded Gazebo development check, source all three environments and run:

```bash
ROS_DOMAIN_ID=75 python3 scripts/verify_lidar_gazebo.py --output /absolute/new-output-folder
```

This script starts its own scene and sends real simulation commands. Do not start another controller or scene in the same domain. It tests a clear 0.5 m segment, an interrupted scan stream with continuing simulator odometry, and a red-obstacle approach. It forwards `/scan` to `/validation/scan` so scan loss can be injected without disabling Gazebo. Processes are stopped on completion/failure. The output directory must not already exist, preserving earlier results.

## Evidence and limits

See `results/lidar-only-2026-11-05/README.md`. All three sequential Gazebo development checks and all 16 new tests passed in WSL Ubuntu 24.04 / ROS Jazzy. Existing 29 baseline tests also passed.

Additional checks used `--empty-world` for an actual obstacle-free endpoint run and `--delay 1.0` for the three-check suite with the existing camera replay relay active. The latter received 348 attacked images and its relay log confirms REPLAY. Both checks passed. The verifier does not route camera content into LiDAR-only control.

On 6 November, the separately installed teammate occlusion package was tested with `--occlusion`. All three Gazebo checks passed; all 319 received attacked images had the configured black ROI. Its 18 package tests, including 6 real DDS tests, also passed without skips. See `results/lidar-only-2026-11-06/README.md` for source revision, overlay setup and evidence. This does not merge the attack branch into the defense branch. `--occlusion` and `--delay` are mutually exclusive.

The three-case suites are sequential checks in one scene, not independent reset trials. Recorded LiDAR corridor clearance is a sensor-derived longitudinal gap to a configured envelope, not simulator ground-truth clearance or a contact measurement. Endpoint overshoot is retained in the logs. No formal collision rate, false alarm rate, attack success rate or repeated-noise statistics are claimed.

Twenty reset-scene trials were subsequently completed, with five expected outcomes out of five in each of four conditions. See `results/lidar-only-2026-11-06/reset-trials/README.md`. Next steps are calibrated stopping parameters, contact/ground-truth measurements, independent teammate review, and the separate Camera–LiDAR monitor.

Independent simulator-pose checks have now also been completed for clean, replay and occlusion obstacle approaches. See `docs/issue-7-acceptance.md`. Add `--ground-truth` to a single-case verifier invocation to record model positions using native Gazebo Python bindings (available in the deployed Harmonic environment). These inputs are evaluation-only. The latest LiDAR test suite has 19 passing tests; earlier 16-test logs remain historical evidence.

## Reset scene repetitions

`scripts/run_lidar_repetitions.py` starts a fresh verifier and scene for each trial. It rotates condition order between repetitions and retains unsuccessful exits rather than stopping after the first failure.

```bash
# Source ROS, TurtleBot3, project, and the separate teammate occlusion overlay first.
python3 scripts/run_lidar_repetitions.py --output /absolute/new-batch-folder --repeats 5
python3 scripts/summarize_lidar_repetitions.py /absolute/new-batch-folder
```

The four conditions are: obstacle with a zero-delay relay, obstacle with a 1 s delay relay, obstacle with black-rectangle occlusion, and an empty-world 4 m route. The three obstacle conditions use the same LiDAR speed, stopping parameters, robot origin, and obstacle scene. The delay/occlusion verifier waits for the manipulation to become active before enabling movement. Each trial has its own Gazebo transport partition. Existing simulations should be closed first to avoid competing rendering load.

The output includes each trial's full scene/controller/attack logs, `runs.json` with exit codes and observed metrics, `summary.json` with outcome counts, and `statistics.json` with means and sample standard deviations. No added noise or explicit seed is configured. Timing variability can therefore reflect execution/sensor delivery rather than an intentional random-noise experiment. These repeated checks still do not provide independent collision or ground-truth clearance measurements.
