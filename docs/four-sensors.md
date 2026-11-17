# Four-sensor bring-up: camera, LiDAR, IMU, wheel speed

The pinned TurtleBot3 Waffle Pi model already contains the IMU and the wheel
joint-state publisher, but the previous experiment documentation only used the
camera and LiDAR. This branch makes all four sensors first-class: it exposes a
dedicated wheel-speed topic, documents the existing IMU, checks liveness, and
records one synchronised CSV row per sample group.

## What is already in the model

At the pinned `turtlebot3_simulations` revision
[`45633014`](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/45633014a14e8f438495b532a723e4ad45cbbd31/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf):

- `imu_link` has an `imu` sensor named `tb3_imu`, `update_rate=200`, topic
  `imu`, with Gaussian angular-velocity and linear-acceleration noise.
- `gz-sim-joint-state-publisher-system` publishes `joint_states` for
  `wheel_left_joint` and `wheel_right_joint`.
- `gz-sim-diff-drive-system` publishes `odom` and `tf`; wheel radius 0.033 m,
  separation 0.287 m.
- The installed bridge YAML
  [`turtlebot3_waffle_pi_bridge.yaml`](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/45633014a14e8f438495b532a723e4ad45cbbd31/turtlebot3_gazebo/params/turtlebot3_waffle_pi_bridge.yaml)
  maps `imu`, `joint_states`, `odom`, `tf`, `clock`, `scan` and
  `camera/camera_info`; `ros_gz_image` bridges `/camera/image_raw`.

So the upgrade does **not** add a second physical IMU. It exposes the existing
one, adds a wheel-speed conversion node, and makes the four streams verifiable
and recordable together. A second independent IMU would require a model change
and a new sensor pose; that is not part of this branch.

Both experiment scenes (`full` and `empty`) spawn the same Waffle Pi model at
the same pose. Camera, LiDAR, IMU and wheel joint-state topics are therefore
identical; only the world model differs. Run the same `wheel_speed` +
`sensor_sync` pipeline for either scene.

## Topics

| Topic | Type | Nominal rate | Source |
|---|---|---:|---|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | 30 Hz | Gazebo camera + `ros_gz_image` |
| `/scan` | `sensor_msgs/msg/LaserScan` | 10 Hz | Gazebo `gpu_lidar` + bridge |
| `/imu` | `sensor_msgs/msg/Imu` | 200 Hz | Gazebo `tb3_imu` + bridge |
| `/joint_states` | `sensor_msgs/msg/JointState` | simulation rate (measure) | Gazebo joint-state plugin + bridge |
| `/wheel_speed` | `sensor_msgs/msg/JointState` | 30 Hz (configurable) | `tb3_sensor_bringup/wheel_speed` |
| `/odom` | `nav_msgs/msg/Odometry` | 30 Hz | Diff-drive plugin + bridge |
| `/tf` | `tf2_msgs/msg/TFMessage` | — | Diff-drive plugin + bridge |
| `/clock` | `rosgraph_msgs/msg/Clock` | — | Gazebo + bridge |

`/wheel_speed` preserves the `/joint_states` header stamp and stores left/right
wheel angular velocity in `velocity` (rad/s). Multiply by `wheel_radius`
(default 0.033 m) for m/s; the sync logger does this and writes both. The node
republishes at 30 Hz by default even if `/joint_states` arrives faster; this
keeps the recorder light. Repeating a stamp is harmless because the sync
buffer ignores duplicates.

## Synchronised recording

`tb3_sensor_bringup/sensor_sync` keeps the latest sample of each of the four
sensors and runs a 20 Hz timer. It writes a row only when:

- all four sensors have arrived;
- the oldest/newest header-stamp span is at most `sync_tolerance` (default
  0.20 simulation seconds, i.e. 0.20 m at 1.0 m/s);
- the newest sample is no older than `staleness` (0.50 s).

The CSV is `~/sensor-results/<UTC-id>/sensors.csv` with one row per group:

```text
stamp, image_stamp, image_width, image_height, image_encoding,
scan_stamp, scan_min_forward, scan_range_max,
imu_stamp, imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz,
imu_qx, imu_qy, imu_qz, imu_qw,
wheel_stamp, left_rad_s, right_rad_s, left_m_s, right_m_s, sync_span_s
```

`scan_min_forward` is the nearest finite return within ±30° of the scan frame's
+X axis; an all-inf scan writes an empty field. Image pixels are not decoded by
this logger; use the bag for full frames. The node also publishes a JSON status
on `/sensor_sync/status`.

This is **latest-sample** synchronisation, not a hardware trigger. The tolerance
is a bound on pairing skew, not a guarantee that every sensor sampled the same
instant. At 200 Hz IMU and 10 Hz LiDAR, the LiDAR dominates the span.

## Build and run

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/turtlebot3_ws/install/setup.bash"
bash scripts/build_sensors.sh
```

Terminal A, scene (full or empty):

```bash
ROS_DOMAIN_ID=33 bash scripts/start_scene.sh full
# wait for SCENE_READY
```

Terminal B, wheel-speed converter and synchronized CSV:

```bash
ROS_DOMAIN_ID=33 bash scripts/run_sensor_sync.sh
```

Terminal C, passive liveness check (does not drive):

```bash
ROS_DOMAIN_ID=33 bash scripts/run_sensor_check.sh 10
# expected: SENSOR_CHECK_PASS
```

Optional bag with all streams and supporting topics:

```bash
ROS_DOMAIN_ID=33 bash scripts/record_four_sensors.sh
# /camera/image_raw /scan /imu /wheel_speed /joint_states /odom /tf /clock
```

Useful diagnostics while the scene runs:

```bash
ros2 topic hz /camera/image_raw /scan /imu /joint_states /wheel_speed
ros2 topic echo /wheel_speed --once
ros2 topic info /imu --verbose
```

## Acceptance

- `SENSOR_CHECK_PASS` with non-zero counts on all four topics.
- No backward header stamps; maximum stamp gap at most 1.0 s per sensor
  (development threshold; the expected rates are much higher).
- `sensors.csv` has rows; `sync_span_s` is at most 0.20 s for accepted rows.
- Wheel speeds are near zero when the robot is stationary, and left/right
  signs follow the commanded turn direction.
- The bag replays: `ros2 bag info <dir>` lists all recorded topics.

## Host tests

The ROS-independent conversion and pairing logic has host tests:

```bash
PYTHONPATH="$PWD/src/tb3_sensor_bringup" \
  python3 -m unittest discover -s src/tb3_sensor_bringup/test -v
```

ROS/Gazebo liveness still requires the Ubuntu VM and the checks above; the host
tests do not prove DDS delivery or sensor rates.

## Limits

- The wheel speed is derived from Gazebo joint velocities, not an independent
  encoder; it inherits the ideal diff-drive behaviour and cannot detect wheel
  slip.
- The IMU is simulated with noise and is not calibrated against a physical
  MPU-9250; do not use it as a metrology reference.
- `/joint_states` and `/imu` are provided by the pinned TurtleBot3 model. If
  either is missing, check `TURTLEBOT3_MODEL=waffle_pi`, the installed bridge
  YAML and the actual `ros2 topic list` before changing this package.
- Image pixels are only in the bag; the CSV logger stores metadata to keep the
  load low.
- The 0.20 s tolerance is a development choice; it covers the 10 Hz LiDAR
  period and the camera jitter observed in the VM, without silently dropping
  whole scans. It is not a statistical guarantee; filter on `sync_span_s`
  when a tighter bound is needed.
