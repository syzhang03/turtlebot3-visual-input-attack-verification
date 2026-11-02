# Camera Occlusion Attack

This package adds a camera-only relay for controlled simulation experiments:

```text
/camera/image_raw -> camera_occlusion -> /camera/attacked_image
                                            |
                                  visual_baseline (image_topic override)
                                            |
                                  /baseline/debug_image
```

`tb3_camera_attack` is independent of `tb3_visual_baseline`. The existing detector,
controller, red obstacle, LiDAR and Gazebo configuration are unchanged. The relay
does not subscribe to LiDAR/odometry or publish robot commands. It implements only
Clean Mode and black rectangular Occlusion Mode; there is no attack scheduling,
frame replay, automatic recovery or defense implementation.

## Image behavior

- **Clean Mode** (`attack_enabled: false`, the default): publish the original
  `sensor_msgs/msg/Image` without conversion. Header, timestamp, frame ID, pixel
  bytes, dimensions, encoding, endianness and row stride are preserved.
- **Occlusion Mode** (`attack_enabled: true`): use `cv_bridge` to convert to BGR8,
  copy the image, and use OpenCV to fill the configured rectangle with black.
  Publish BGR8 with the original header copied, including its original timestamp
  and frame ID. Width and height are unchanged. Outside the rectangle, BGR pixel
  values are unchanged from the converted input. For an RGB8 input, output byte
  channel order changes to BGR8 while colors outside the rectangle are preserved.
- Both image endpoints use `qos_profile_sensor_data` (best effort, volatile).
  A subscriber must use compatible QoS. This is an incoming-frame-driven relay;
  it does not synthesize frames or rewrite timestamps.
- Conversion/processing errors log an error and drop that frame. They do not
  produce a clean replacement or a new timestamp. The existing baseline's sensor
  timeout still applies if valid frames stop arriving.

## Parameters and default ROI

| Parameter | Default | Meaning |
|---|---|---|
| `input_topic` | `/camera/image_raw` | Input Image topic; read-only after startup |
| `output_topic` | `/camera/attacked_image` | Output Image topic; read-only after startup |
| `attack_enabled` | `false` | False = Clean Mode; true = Occlusion Mode; runtime configurable |
| `mask_x` | `0.2` | Normalized left edge; runtime configurable |
| `mask_y` | `0.25` | Normalized top edge; runtime configurable |
| `mask_width` | `0.6` | Normalized width; runtime configurable |
| `mask_height` | `0.75` | Normalized height; runtime configurable |
| `use_sim_time` | `true` in YAML | Standard ROS simulation clock parameter |

Use floating-point values for all four mask parameters, including `0.0` and `1.0`.
The default rectangle is derived from the actual `vision.py` detector:

```text
x0 = int(image_width * 0.2)
x1 = int(image_width * 0.8)
y0 = int(image_height * roi_top)  # baseline.yaml: roi_top = 0.25
ROI = image[y0:, x0:x1]
```

At 320 x 240, the default mask is columns `[64, 256)` and rows `[60, 240)`.
At 640 x 480 it is columns `[128, 512)` and rows `[120, 480)`.
This covers the detector's entire default search region, **not an assumed central
obstacle bounding box**. Tests place synthetic red obstacles at four different
positions inside that region and invoke the existing detector before/after
occlusion. This does not establish the obstacle's location in a live Gazebo image.
Inspect the camera/debug images before choosing a smaller mask. If the baseline's
`roi_top` is changed, adjust the attack parameters explicitly; they do not track it
automatically.

For any configured rectangle, pixel bounds are:

```text
x0 = int(width * mask_x)
x1 = min(width, int(width * (mask_x + mask_width)))
y0 = int(height * mask_y)
y1 = min(height, int(height * (mask_y + mask_height)))
```

The rectangle uses half-open bounds. A subpixel-sized rectangle that rounds to an
empty region leaves the pixels unchanged. Coordinates must be finite;
`0 <= x,y < 1`, `0 < width,height <= 1`, and the rectangle must fit inside the image.

Invalid numeric coordinates at startup produce a warning and reset **all four**
mask parameters to the valid detector-ROI defaults. Invalid runtime coordinate
updates are rejected with a reason and leave the previous settings active, so the
node continues processing. ROS parameter type errors are handled by ROS's type
validation. Input and output topics resolving to the same topic, including through
remapping, are rejected at startup to prevent a relay feedback loop.

## Build on Ubuntu 24.04 / ROS 2 Jazzy

Run from an Ubuntu-local checkout of this repository on `camera-occlusion-attack`:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select tb3_visual_baseline tb3_camera_attack
source install/setup.bash
export ROS_DOMAIN_ID=33
```

Use the same domain ID in every experiment terminal. Dependencies are declared in
the new package's `package.xml`, including `rclpy`, `rcl_interfaces`, `sensor_msgs`,
`cv_bridge`, `python3-opencv`, `python3-numpy` and launch dependencies. Use the ROS
environment's Python and distribution packages for ROS execution.
The existing `scripts/build_baseline.sh` selects only the baseline package, so use
the explicit two-package build above.

## Run the relay

After sourcing ROS and both workspaces in a terminal with the same domain ID:

```bash
# Clean Mode
ros2 launch tb3_camera_attack camera_occlusion.launch.py

# Alternatively, stop the clean relay first and start Occlusion Mode:
ros2 launch tb3_camera_attack camera_occlusion.launch.py attack_enabled:=true
```

The launch file starts only the relay. It loads the installed YAML; its
`attack_enabled` launch argument overrides that YAML value and defaults to false.
An alternative configuration can be supplied with `params_file:=/absolute/file.yaml`.

Direct invocation with explicit parameters is also supported:

```bash
ros2 run tb3_camera_attack camera_occlusion --ros-args \
  --params-file "$PWD/src/tb3_camera_attack/config/camera_occlusion.yaml" \
  -p attack_enabled:=true \
  -p mask_x:=0.2 -p mask_y:=0.25 -p mask_width:=0.6 -p mask_height:=0.75
```

Toggle a running relay without restarting it:

```bash
ros2 param set /camera_occlusion attack_enabled true
ros2 param set /camera_occlusion attack_enabled false
```

Example of a smaller, offset rectangle; apply in this order so every intermediate
rectangle remains valid:

```bash
ros2 param set /camera_occlusion mask_width 0.25
ros2 param set /camera_occlusion mask_height 0.3
ros2 param set /camera_occlusion mask_x 0.55
ros2 param set /camera_occlusion mask_y 0.1
```

For multiple simultaneous changes, a ROS client can use
`set_parameters_atomically`; the node validates the proposed rectangle as a whole.
The example offset rectangle is a configuration demonstration, not a claim about
the obstacle location or attack effectiveness.

## Connect the existing baseline

Start/reset the simulation and obstacle as documented in `baseline.md`. Start the
relay before the baseline. In a separately sourced terminal, from repository root:

```bash
export ROS_DOMAIN_ID=33
ros2 run tb3_visual_baseline baseline --ros-args \
  --params-file "$PWD/configs/baseline.yaml" \
  -p image_topic:=/camera/attacked_image -p enabled:=false
```

Observe `/camera/attacked_image` and `/baseline/debug_image` before a moving trial.
For the obstacle-approach trial, stop the observe process, reset the scene as
needed, and run the same command with `-p enabled:=true -p goal_distance:=4.0`.
There must be only one baseline controller / `/cmd_vel` publisher:

```bash
ros2 topic info /cmd_vel --verbose
ros2 topic info /camera/attacked_image --verbose
ros2 param get /visual_baseline image_topic
```

No changes to `configs/baseline.yaml` are needed. The existing baseline shell
script does not forward extra ROS arguments; use the direct command above.
Its `image_topic` takes effect at startup, so restart the baseline to change it.

The baseline latches `OBSTACLE_STOP`. Turning on occlusion after an obstacle stop
does not resume motion. To test concealment, activate occlusion before the first
obstacle detection. A reduced red ratio is evidence of altered perception, not by
itself evidence of collision or measured clearance. Reset each trial and retain
timeouts, failures and unsuccessful attacks in the results.

## Tests and validation record

Synthetic image tests (including actual existing detector compatibility), adapter
tests and the existing baseline suite can be run together from repository root:

```bash
PYTHONPATH="$PWD/src/tb3_camera_attack:$PWD/src/tb3_visual_baseline" \
  python3 -B -m pytest src/tb3_camera_attack/test src/tb3_visual_baseline/test -v -ra
```

For package tests through ROS tooling:

```bash
colcon test --packages-select tb3_camera_attack --event-handlers console_direct+
colcon test-result --verbose
```

The detector compatibility test requires the baseline package on `PYTHONPATH` or
installed in the sourced workspace. ROS tests require real `rclpy`, `cv_bridge`
and message packages. Missing ROS modules cause explicit skips, not simulated
passes. The ROS tests publish synthetic images through actual DDS and cover:

- Clean Mode message equality.
- Occlusion pixels, unchanged outside pixels, dimensions and original header.
- RGB8 conversion to BGR8 without changing colors outside the rectangle.
- Invalid runtime coordinates rejected while processing continues.
- Invalid startup coordinates replaced with valid defaults.
- A valid atomic rectangle update and switching back to Clean Mode.

Validation performed on **2026-11-03**, Windows, Python 3.14.6, OpenCV 5.0.0,
NumPy 2.4.6, pytest 9.0.3, using an ignored project-local `.venv`:

- **22 passed, 6 skipped**: 12 new image/adapter tests and all 10 existing baseline
  tests passed. The 6 real ROS tests skipped because `rclpy` is unavailable.
- Image tests verify black fill, exact unchanged outside pixels, input immutability,
  output shape/type, full-frame edges, subpixel rounding, invalid values, and default
  ROI bounds at multiple resolutions including odd dimensions.
- Adapter tests use a clearly identified bridge double to check message/header
  handling. They do not validate actual `cv_bridge` or ROS serialization.
- The Python wheel built successfully. Its package metadata, executable entry point,
  installed ament resource, configuration and launch files were checked. Python
  syntax, XML and YAML also passed static checks.
- `colcon build`, launch execution, real ROS tests and Gazebo trials were **not run**
  on this host. Ubuntu Jazzy dependency compatibility, DDS delivery, actual
  header preservation through ROS, and invalid-parameter node survival still need
  the real ROS tests above. Local OpenCV results do not replace that validation.

No code changes to existing baseline/control/scene files are required.
