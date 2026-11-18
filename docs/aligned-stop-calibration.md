# Align the visual and LiDAR trigger locations

This experiment profile uses the clean-scene distance at which the existing red contour / ROI area ratio crosses **0.4**. The visual stopping and verification thresholds both become 0.4. LiDAR stopping uses the calibrated fixed corridor clearance `D`; it no longer adds reaction/braking terms to D. The default LiDAR-only code still uses the original dynamic formula unless the fixed profile is selected.

`D` is TF-transformed forward-corridor clearance with the configured front extent subtracted. It is neither odometry displacement nor the raw minimum laser range, and it is not the final physical clearance after braking.

## Measure in the current clean scene

Stop the previous controller, relays and scene with Ctrl+C. Reset by launching a fresh scene, then wait for SCENE_READY:

```bash
ROS_DOMAIN_ID=33 bash ~/camera-lidar-attack/scripts/start_scene.sh
```

In another terminal, run only this controller (no attack relays):

```bash
ROS_DOMAIN_ID=33 bash ~/camera-lidar-attack/scripts/run_stop_calibration.sh
```

The calibration approach uses raw `/camera/image_raw` and `/scan`, speed 0.3 m/s, visual threshold 0.4, and 0.06 s pairing tolerance. `calibration_mode=true` suppresses consistency-stop requests only; visual stopping, the dynamic LiDAR guard, sensor freshness checks and command conflict checks remain active. Wait for VISUAL_STOP, wait at least 3 seconds for stopped odometry and paired records, then Ctrl+C. A timeout or earlier LiDAR stop is rejected rather than accepted as calibration.

Generate the profile from that run:

```bash
python3 ~/camera-lidar-attack/scripts/calibrate_stop_point.py --latest
```

The script interpolates between the paired observations immediately below/above 0.4. It rejects a missing bracket, timestamp rollback/duplication, nonfinite clearance, a gap over 0.3 s, or a nonapproaching bracket. It reports the measured bracket and pairing offsets. It creates `configs/aligned_stop.yaml`, backs up any previous profile, and saves the source record and derivation in the calibration run. Calibration logs are separate from formal trials.

The consistency band contains this same crossing but allows for sampling: its edges cover the measured clearance bracket plus travel during the formal pairing tolerance (1.0 m/s × 0.06 s), rounded outward to centimetres. Inside the band, consistency is ambiguous; outside, three same-direction disagreements can latch ANOMALY_STOP. The independent visual and LiDAR stop requests remain immediate. This band is not a statistical confidence interval and does not cover every scene, noise or delivery-delay error. Repeat clean validation if geometry/ROI changes or false alarms occur.

## Run the four matched conditions

After calibration, reset the scene again. For each condition, start exactly one relay invocation in one terminal:

```bash
# Choose ONE per reset trial:
ROS_DOMAIN_ID=33 bash ~/camera-lidar-attack/scripts/run_joint_attack.sh clean
ROS_DOMAIN_ID=33 bash ~/camera-lidar-attack/scripts/run_joint_attack.sh camera 1.0
ROS_DOMAIN_ID=33 bash ~/camera-lidar-attack/scripts/run_joint_attack.sh lidar 0.0 1.0
ROS_DOMAIN_ID=33 bash ~/camera-lidar-attack/scripts/run_joint_attack.sh joint 1.0 1.0
```

After both relays report ready, use the same controller in another terminal:

```bash
ROS_DOMAIN_ID=33 bash ~/camera-lidar-attack/scripts/run_joint_controller.sh drive
```

This helper defaults to `aligned` and requires `configs/aligned_stop.yaml`. It copies the exact profile into each run directory. Startup reports `LiDAR stop: mode=fixed; threshold=...; speed=1.0`. Both visual thresholds are 0.4, the endpoint stop is disabled, and the distance band is common to all four conditions. First validate clean at 1.0 m/s, then freeze the profile for the attack trials. Reset the scene and restart relays/controller between runs. For the reported unchanged scene, see the [recorded profile and four-trial guide](../results/camera-lidar-2026-11-14/README.md).

`run_joint_controller.sh drive legacy` explicitly selects the previous dynamic-distance and 0.06 verification configuration. Old results must not be pooled with the new profile as matched comparisons. `summarize_joint_trial.py` labels the selected profile and checks the logged fixed threshold.

Moving the verification boundary nearer the obstacle changes the defense being evaluated. Camera or LiDAR may request an ordinary obstacle stop before three disagreement pairs accumulate. Therefore no ANOMALY_STOP alone does not prove failure, and a stop alone does not prove attack detection. Report stop reason, commands, independent final clearance and contact separately. Matching trigger locations does not guarantee matching final stopping positions.

## Local checks

```bash
python3 -m unittest discover -s tests -p test_stop_calibration.py -v
python3 -m unittest discover -s src/tb3_lidar_baseline/test -v
python3 -m unittest discover -s src/tb3_camera_lidar_monitor/test -v
```

Source the built ROS workspace before ROS tests. Windows checks can exercise pure geometry/calibration but cannot establish Gazebo or DDS behavior; the new aligned profile needs the VM calibration and clean validation above.
