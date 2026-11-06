# Camera LiDAR development evidence

Date: 6 November 2026. Runtime: existing WSL Ubuntu 24.04 / ROS Jazzy / Gazebo Harmonic, pinned TurtleBot3 dependencies. Code starts from main merge `9cf784f`, with the shared LiDAR readiness hook, new monitor package, launcher/configuration, and verifier extensions.

## Calibration and frozen validation

The clean `calibration/` run approached at 0.3 m/s with anomaly mitigation disabled and visual/LiDAR stopping enabled. It ended at VISUAL_STOP. Its 115 pairs were used to fit the ambiguity band; `calibration/band.json` records the calculation. No attack data was used to choose the band. Subsequent validation used the fixed 2.57–2.91 m band at 1.0 m/s with three-pair confirmation.

| Run | Final state | Consistency alert | Simulator-pose separation lower bound (m) |
|---|---|---|---:|
| Calibration, clean, 0.3 m/s | VISUAL_STOP | none | 0.625440 |
| Clean, 1.0 m/s | OBSTACLE_STOP | none | 0.921505 |
| 1 s replay, 1.0 m/s | ANOMALY_STOP | VISUAL_MISS, count 3 | 2.153244 |
| Black ROI, 1.0 m/s | ANOMALY_STOP | VISUAL_MISS, count 3 | 2.131391 |

All final command speeds and measured forward speeds were zero. In the replay run, the first confirmed pair had 2.338907 m LiDAR clearance and red ratio 0.038672. In the masking run, the first confirmed pair had 2.305611 m LiDAR clearance and red ratio zero. Each pair was consumed once. The nominal emergency threshold remained 1.15 m, unchanged from LiDAR-only.

`empty/` records a separate empty-world 4 m route. Consult its summary and pair CSV for endpoint completion and absence of alert; it is a single development check, not a false-alarm-rate study.

## Tests

- `monitor-tests.log`: 12 passed, no skips (6 pure monitor tests, 6 actual ROS DDS tests).
- `lidar-regression.log`: 19 passed, no skips.
- `visual-regression.log`: 29 passed.
- Three project packages built successfully in the Linux build workspace.

DDS checks cover normal agreement, both disagreement directions, camera loss, and immediate LiDAR danger priority. Geometry/state tests cover duplicate consumption, ambiguity, gaps, direction changes and timestamp rollback rejection.

## Interpretation and ownership

Raw scene/controller/attack logs, pairs and independent simulator-pose CSVs are retained per run. The same 0.30 m evaluation enclosure and box geometry as Issue #7 provide conservative separation bounds, not exact mesh/contact measurements. The controller never receives ground truth.

Early run config.json files retain the inherited base revision label `lidar-only-v1`; visual-config.json, pairs.csv, process logs and summary.controller identify the actual fusion executable and settings. The final implementation labels fusion metadata separately. No raw files were rewritten to hide this development history.

This first monitor version intentionally preserves conservative visual-stop requests; it may increase unnecessary stops. Camera ROI/corridor correspondence is calibrated for a fixed centered red obstacle, not a general geometric projection. Alerts report inconsistency, not proven attacks. A full matched repeated comparison against Camera-only and LiDAR-only is separate work.

Codex implemented this monitor and ran the recorded checks. The HSV detector, trusted LiDAR guard and teammate's unchanged masking relay are reused with attribution. No independent human review, statistical superiority or final Issue #9 acceptance is claimed here.
