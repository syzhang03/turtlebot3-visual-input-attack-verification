# Camera–LiDAR attack experiment guide

The current English guide is the **[14 November experiment README](../results/camera-lidar-2026-11-14/README.md)**.

It covers historical camera-frame replay with current message headers; forward-sector LiDAR range inflation; the four conditions `clean`, `camera`, `lidar`, `joint`; the recorded 1.0 m/s profile; results and evidence limitations; exact reproduction commands; and attacked topic interfaces for teammate defenses.

For a changed scene or sensor geometry, follow [stop-point calibration](aligned-stop-calibration.md). For the reported scene, the README explains how to activate [the frozen profile](../configs/aligned_stop_20261114.yaml) and validate clean first.

The root project README and historical evaluations are retained. This extension allows LiDAR compromise as well as camera compromise; it does not inherit the original assumption that LiDAR is trusted.
