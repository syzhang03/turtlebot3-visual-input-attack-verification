# Add calibrated Camera LiDAR consistency monitoring and protective control

Closes #9

## Behavior

Add an integrated Camera–LiDAR controller that consumes attacked images and trusted scans, confirms directional disagreement across distinct timestamp pairs, and requests a latched anomaly stop. Keep the existing nominal visual stopping threshold and the same trusted LiDAR emergency guard. Missing required camera/pair inputs stop motion instead of being interpreted as agreement.

Add bounded one-to-one pairing, an ambiguity band fitted on a clean scene, per-pair JSON diagnostics/CSV logs, startup configuration, build/run helpers, and real ROS integration tests. LiDAR-only retains its own executable and defaults; a no-op readiness hook enables reuse without duplicating emergency control.

## Verification

- Three project packages built in WSL Ubuntu 24.04 / ROS Jazzy.
- New monitor: 12 tests pass, including 6 real DDS tests.
- LiDAR regression: 19 tests pass; visual regression: 29 tests pass.
- A clean 0.3 m/s calibration fitted a 2.57–2.91 m ambiguity band for verification ratio 0.06. The nominal visual stopping ratio remains 0.4.
- Frozen-parameter Gazebo checks at 1.0 m/s: clean obstacle approach stops normally without anomaly alert; 1 s replay and masking trigger VISUAL_MISS confirmation and ANOMALY_STOP; empty-world 4 m route reaches its endpoint without an anomaly alert.
- Independent simulator-pose clearance lower bounds: 0.921505 m clean, 2.153244 m replay, 2.131391 m masking. Raw files are retained under results/camera-lidar-2026-11-06.
- Repeated validation on 7 November: all 44 frozen reset trials and 9 near-obstacle fault trials passed. All 15 attacked obstacle trials alerted; all 15 empty tasks completed without alert. Minimum sampled independent pose bound across the 53 trials was 0.748587 m. All pairing audits passed; the confirming three-pair sequences span 0.2 simulation seconds. Full evidence is in results/camera-lidar-2026-11-07.

## Scope and limitations

This is a fixed centered red-obstacle monitor with coarse ROI/corridor correspondence, not general pixel-to-point-cloud fusion. The development checks and repeated validation do not establish statistical superiority or prove that fusion is necessary. Retaining visual stops can cause unnecessary stops. A full matched comparison against Camera-only and LiDAR-only is the next evaluation issue.

AI assistance: Codex implemented and tested this change, reusing the team's HSV detector, LiDAR guard, scene and unchanged camera attack packages. Independent teammate review and reproduction remain required before merge. No human review is claimed by the automated results.
