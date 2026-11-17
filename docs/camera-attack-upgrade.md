# Camera attack upgrade

Only `hou_meiqi_camera_attack` is modified. Latest main was merged normally with
no conflicts. See [INTEGRATION.md](../INTEGRATION.md) for exact team interfaces,
QoS, ROS tests and limitations. Old executable, rectangle defaults, attack_enabled
and `/camera/attacked_image` remain supported. No detector, controller or formal
result has been edited. The legacy occlusion document describes the original
version; this document supersedes its modes/QoS statements for the upgraded node.

## Runtime settings

`attack_enabled=false` always bypasses processing; `attack_mode=clean` also bypasses.
Modes: occlusion, intermittent_occlusion, gaussian_noise, brightness, contrast,
color_shift. These are digital input perturbations, not FGSM or physical attacks.

| Settings | Defaults and meaning |
|---|---|
| mask_x, mask_y, mask_width, mask_height | .2, .25, .6, .75; normalized top-left and size |
| start_s, duration_s | 0.0, -1.0; first input timestamp is time zero; -1 = unlimited |
| period_s, on_s | 2.0, 1.0; intermittent active interval per period |
| noise_sigma, random_seed | 0.0, 0; Gaussian standard deviation in 8-bit units |
| brightness_delta | 0.0; signed offset, clipped to [0,255] |
| contrast_factor | 1.0; scale about 127.5, range [0,10] |
| shift_b, shift_g, shift_r | 0.0; signed channel offsets, BGR names even for RGB input |
| preserve_encoding | false; legacy BGR8 output, optional original 8-bit encoding |

Use double-valued ROS parameters for all real settings. Updates should use
`set_parameters_atomically` when changing dependent ROI/period values together.
Invalid updates are rejected without side effects. Seed changes restart the RNG;
mode switches alone do not rewind it. Restart for each matched-seed trial.
Alpha channels remain unchanged. Pixel math clips before conversion to uint8.
Default masks are still validated by the original tests. To place the rectangle
at center (cx,cy), use mask_x=cx-width/2 and mask_y=cy-height/2; the entire ROI must
fit within the image. Empty subpixel ROIs do not magically cover an obstacle.
Restart the relay after a scene timestamp reset; backward stamps are dropped.

## Reproducible initial conditions

Installed configs include clean, occlusion_10, occlusion_30, occlusion_60x75,
noise_0/5/15/30 (seed 7), darker (-50) and brighter (+50). These are exploratory
strengths, not optimized or validated attacks. The small rectangles are centered
in the image; if they miss the red obstacle, retain the failed attack.

For each config, reset the scene, robot, controller and relay. Use the unchanged
scene, speed, thresholds and sensors in INTEGRATION.md. Repeat randomized trials
with explicit seeds, matching clean/attack scene conditions. For a fresh trial:

```bash
mkdir -p "$HOME/meiqi-camera-results"
bash scripts/run_camera_attack_trial.sh \
  src/tb3_camera_attack/config/noise_15.yaml \
  "$HOME/meiqi-camera-results/noise15-seed7-$(date +%Y%m%dT%H%M%S)"
```

Start exactly one controller separately, initially disabled/observe. This script
replaces all other camera relays. It records images, scan, odometry, clock,
commands and consistency diagnostics, configuration, environment and code version.
It does not automate scene reset or assert success. Replay also publishes the
same topic and must be stopped. Publisher competition suppresses this relay.

Read `red_ratio`/`red_area`, detector decisions and anomaly information from the
existing controller/monitor logs (`events.csv`, `pairs.csv`). Bag timestamps and
commands retain first stop and displacement evidence. Use the existing
`verify_lidar_gazebo.py --ground-truth` for supported default-mask safety trials.
Independent clearance and collision are unavailable for arbitrary perturbation
recordings unless that evaluator is adapted and run. Mark unavailable metrics
Not Tested; do not substitute a perception miss for collision evidence.

## Validation boundary

Windows pure-Python/OpenCV tests and adapter doubles do not validate ROS DDS,
cv_bridge, ROS runtime parameter enforcement, Gazebo or robot safety. The new
real Monitor test and retained ROS tests are skipped on this host. `INTEGRATION.md`
contains the real Ubuntu commands. No VMware SSH target is configured, so VM
deployment, colcon build/test and simulator campaigns remain Not Tested.
