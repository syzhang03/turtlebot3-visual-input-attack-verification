# Latest update — 14 November 2026

## Camera and LiDAR attacks with an aligned stopping baseline

The **[experiment README](results/camera-lidar-2026-11-14/README.md)** explains the two attacks, the four retained trials, results and evidence limits, exact reproduction commands, and interfaces for teammates developing defenses.

- Reused the houses, road, white stop line and fixed red box scene with TurtleBot3 Waffle Pi, ROS 2 Jazzy and Gazebo Harmonic.
- Added a LiDAR relay that inflates valid ranges by up to 1.0 m within the forward ±30-degree sector, alongside the existing 1.0-second camera historical-frame replay.
- Aligned visual stopping and consistency at red contour/ROI ratio 0.4 with a calibrated fixed LiDAR trigger of 0.788 m. All four trials used 1.0 m/s and disabled endpoint stopping.
- Added reliable camera-input configuration, native/ROS stream diagnostics, attack/command audits, calibration tools and a passive post-stop collision-geometry check.

| Condition | Stop state | Forward displacement at first logged zero odometry speed | Outcome evidence |
|---|---|---:|---|
| No attack | `VISUAL_STOP` | 2.9653 m | Operator reported no contact |
| Camera only: 1 s replay | `OBSTACLE_STOP` | 3.0263 m | Operator reported no contact |
| LiDAR only: +1 m | `VISUAL_STOP` | 2.9733 m | Operator reported no contact |
| Joint attack | `OBSTACLE_STOP` | 3.7083 m | Later geometry check: current gap at least 6.42 cm; earlier contact unknown |

The joint attack moved the stopping position forward by approximately **0.743 m** relative to the clean run. Both injections were verified in the reported logs, but **collision success was not established**. These are individual development trials, not a statistical success-rate study. A faster-speed experiment was discussed but was not completed or included.

Start with the [English reproduction and defense handoff](results/camera-lidar-2026-11-14/README.md#6-reproduce-on-another-ubuntu-machine), [recorded profile](configs/aligned_stop_20261114.yaml) and [reported results/provenance](results/camera-lidar-2026-11-14/reported-results.json). Full original VM logs are still to be imported; the published summaries explicitly identify their operator-provided source.

The original project README is unchanged. LiDAR and joint compromise extend its original camera-only threat model. The previous [October camera-only update](docs/archive/2026-10-30-update-notes.md) and [October evidence](results/delay-replay-2026-10-30/README.md) describe a different controller and must not be pooled with these trials.
