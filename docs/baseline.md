# Camera-based forward-navigation baseline

This initial implementation drives TurtleBot3 Waffle Pi along its initial heading, stops at an odometry-defined endpoint or on a red visual obstacle, and records commands and observations. It is a controlled-scene prototype, not a general obstacle detector. LiDAR does not participate in its controller. Camera attacks, a cross-sensor monitor, and the final four-group evaluation are not implemented.

The baseline combines visual perception and control in one node at this stage. A future camera relay can connect through the configurable `image_topic`. Separating the perception and control nodes remains a possible integration change relative to the proposal.

## Environment

The contributor verified Ubuntu 24.04.4 LTS, ROS 2 Jazzy, Gazebo Sim 8.15.0 (Harmonic), and the `waffle_pi` model in VMware. The VM has 4 virtual CPUs and 8 GiB RAM. Setting `LIBGL_ALWAYS_SOFTWARE=1` resolved the reported Gazebo flickering. Exact source dependency revisions are in [`../configs/turtlebot3.repos`](../configs/turtlebot3.repos); the contributor's `git status --short` output showed no changes in those three dependency repositories.

The previously measured camera receive rate ended at about 3.23 Hz with a maximum receive interval of 1.352 seconds; LiDAR ended at about 3.51 Hz. These are CLI receive-side observations, not configured simulation-time rates or verified dropped-frame counts. Software-rendering performance and sensor timing remain limitations for formal experiments.

## Build and test

Clone the repository into an Ubuntu-local filesystem. From its root:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash scripts/build_baseline.sh
PYTHONPATH="$PWD/src/tb3_visual_baseline" python3 -m unittest discover -s src/tb3_visual_baseline/test -v
```

The contributor provided an Ubuntu test transcript showing all 10 tests passing (7 control tests and 3 synthetic-image tests). Real Gazebo trials were run manually by the contributor; see the evidence index under `results/`. Windows checks alone do not validate ROS execution. Set `TB3_WS` for the helper scripts if the dependency workspace is in another location.

## Interfaces

| Topic | Message type | Direction |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Input, SensorData QoS |
| `/odom` | `nav_msgs/msg/Odometry` | Input, SensorData QoS |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Output, sole velocity publisher |
| `/baseline/debug_image` | `sensor_msgs/msg/Image` | Output, SensorData QoS, original image header |

Stop teleop and other command publishers before running this node. Observe mode also publishes zero velocity, so it must not run concurrently with another baseline controller. Check with `ros2 topic info /cmd_vel --verbose`.

## Start a reproducible scene

Stop the previous controller, then stop the previous simulator. In a fresh terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID=33
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch turtlebot3_gazebo empty_world.launch.py x_pose:=0.0 y_pose:=0.0
```

Verify a single robot starts near the world origin pointing along +X and no old obstacles remain. All experiment terminals must use the same domain ID (33 for this contributor). Do not restart only the controller and assume the physical scene has reset.

For the obstacle condition, source ROS and the dependency workspace in another terminal, set domain 33, change to this repository's root, and run:

```bash
ros2 run ros_gz_sim create -file "$PWD/worlds/red_obstacle.sdf" \
  -name baseline_red_obstacle -x 4.0 -y 0.0 -z 0.0
```

Check that creation succeeded and the red box lies ahead of the robot. The box is 0.3 m along X, 0.5 m along Y and 0.5 m tall. Its link is raised by 0.25 m so its bottom sits at ground level. Only one Gazebo world should be running; add `-world <actual-name>` if world selection is ambiguous.

## Observe before driving

```bash
bash scripts/run_baseline.sh observe
```

Expected state: `DISABLED`, zero velocity. In a separate ROS-sourced terminal with domain 33, view `/baseline/debug_image` using rqt. A cyan rectangle marks the central ROI; a green rectangle marks the largest detected red component. The HSV ranges are H=0..10 and 170..179, S>=100, V>=70; the ROI is the central 60% of image width, from 25% of image height to the bottom. The ratio is contour area divided by ROI area, not distance.

The contributor encountered blank rqt windows and recovered using a fresh temporary configuration and X11 backend:

```bash
RQT_CHECK_CONFIG="$(mktemp -d)"
XDG_CONFIG_HOME="$RQT_CHECK_CONFIG" QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 \
  ros2 run rqt_image_view rqt_image_view /baseline/debug_image
```

Close the viewer before measured trials to keep workload comparable. For a moving obstacle-approach trial, the initial ratio must be below 0.0600. A box at 1 m caused an immediate stop, and a later displayed ratio near 0.0632 also exceeded the threshold. These development observations are retained in the results narrative; they are not moving-approach successes.

## Run the clear-route condition

Restart the empty world without a box. Stop observe mode and the viewer, then run from the repository root:

```bash
bash scripts/run_baseline.sh drive
```

The defaults are 0.05 m/s and 1 m target displacement. Expected transition: `WAITING -> RUNNING -> GOAL_REACHED`. The contributor observed a stop with the terminal reporting 1.002 m. Verify actual stopping, not just the terminal state.

## Run the obstacle condition

Restart the scene and spawn the box at X=4 m for every trial. After stopping observe mode and the viewer:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=33
ros2 run tb3_visual_baseline baseline --ros-args \
  --params-file "$PWD/configs/baseline.yaml" \
  -p enabled:=true -p goal_distance:=4.0
```

The 4 m goal is an explicit override; the YAML default remains 1 m for the clear-route test. Expected transition: `RUNNING -> OBSTACLE_STOP`. Leave logging active for about 3 seconds after stopping, then Ctrl+C. Record whether contact occurred and save a screenshot before resetting the scene. A new supplemental trial must retain its own identifier; never present a later screenshot as evidence of an earlier trial.

## Logs and interpretation

Each invocation writes `~/baseline-results/<UTC-run-id>/config.json` and `events.csv`. Inspect a run with:

```bash
python3 scripts/summarize_baseline.py ~/baseline-results/ACTUAL_RUN_ID
```

The CSV records simulation time, monotonic wall elapsed time, state, forward displacement, visual ratio, commands, odometry velocity, receive ages and source message timestamps. `distance_m` is projected odometry displacement from this node's initial pose, not obstacle clearance. Terminal stop-trigger distance and final CSV displacement can differ due to subsequent motion. Sample count depends on how long the node remains running after stopping and is not the number of independent trials.

Stop states latch until process restart. A stale image or odometry stream after motion starts produces `SENSOR_TIMEOUT`; its 2-second threshold uses monotonic wall time. Repeated source timestamps do not refresh receive ages. Time moving backwards, conversion errors, invalid odometry, or exceeding the 120-second simulation-time motion limit can stop a run. These execution safeguards are not attack detectors. Apply identical safeguards in all eventual comparison groups and record timeout outcomes separately.

Ctrl+C attempts to publish zero velocity before shutdown. An abrupt process kill or host failure cannot guarantee final command delivery. No collision, minimum-clearance, detection-latency or braking-latency evaluator is implemented here. Zero measured forward velocity and manual no-contact observations are distinct pieces of evidence; neither supplies an independently measured clearance.

## Scope of validation

The clear-route trial and repeated fixed red-box approaches are preliminary development validation. There is no random-noise study, no formal robustness claim, and no claim that attacks or defenses have been evaluated. Full experiments will require matched scene, initial state, parameters, seed, code revision, source revisions and all outcomes, including failures.

AI assistance and human verification are described in [`ai-usage.md`](ai-usage.md).

References: [TurtleBot3 simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/), [Gazebo model spawning](https://gazebosim.org/docs/harmonic/ros2_spawn_model/).
