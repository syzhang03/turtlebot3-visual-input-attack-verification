# 本版更新内容

This document summarizes the implemented changes and validation evidence in the `wu-haonan-visual-baseline` branch. The shared project proposal is in [README.md](README.md).

## Implementation delivered

With Codex assistance, I prepared and validated the first camera-based forward-navigation baseline for TurtleBot3 Waffle Pi:

- Added the [`tb3_visual_baseline` ROS 2 package](src/tb3_visual_baseline), including an OpenCV HSV detector for a red obstacle in the central camera region.
- Connected camera images and odometry to a controller that drives forward, stops at the configured endpoint, and latches a stop when the visual obstacle threshold is reached.
- Added observation mode and a debug-image output to inspect detection before enabling motion.
- Added sensor timeout and execution safeguards, plus per-run configuration and CSV logging for states, visual ratios, displacement, commands and odometry velocity.
- Supplied [baseline parameters](configs/baseline.yaml), a [red-box simulation model](worlds/red_obstacle.sdf), [build/run and log-summary scripts](scripts), and [pinned TurtleBot3 dependency revisions](configs/turtlebot3.repos).

The implementation currently combines visual perception and control in one node. Its image topic is configurable for later integration. LiDAR does not control this baseline.

## Environment setup and troubleshooting

I operated the Ubuntu virtual machine and verified Ubuntu 24.04.4 LTS, ROS 2 Jazzy, Gazebo Sim 8.15.0 (Harmonic), and the Waffle Pi model. The VM used 4 virtual CPUs and 8 GiB RAM.

I investigated Gazebo flickering and blank rqt image windows with assistance. Software rendering resolved the reported Gazebo display issue; a fresh rqt configuration and X11 backend restored image viewing. I documented these steps together with scene reset, obstacle placement and execution instructions in [the baseline guide](docs/baseline.md).

## Tests and simulation trials completed

- Ran the unit tests in Ubuntu and supplied a terminal transcript showing **10 tests passed**: 7 control tests and 3 synthetic-image tests.
- Ran clear-route trials with a 1 m endpoint. The two retained runs reached `GOAL_REACHED`, with final logged forward displacements of approximately 1.0004 m and 1.0021 m.
- Ran four moving approaches to the fixed red obstacle. All four reached `OBSTACLE_STOP`, with final logged forward displacements of approximately 0.9345 m, 0.9339 m, 0.9347 m and 0.9440 m.
- Reported no contact for the moving obstacle approaches and supplied screenshots for three of them. All six main runs ended with a logged forward odometry velocity of 0.0 m/s.

These displacement values are measured from the robot's initial pose; they are not obstacle-clearance measurements. These were preliminary fixed-scene development checks.

## Evidence and handoff

I exported the Ubuntu run data and supplied observations and screenshots. With Codex assistance, the submission was organized into:

- [The trial evidence index](results/baseline-2026-10-27/README.md), linking results, screenshots and evidence limitations.
- [All 12 original timestamped run directories](results/baseline-2026-10-27/runs), including observation-only and immediate-stop runs.
- [An aggregate CSV summary](results/baseline-2026-10-27/summary.csv), per-run summaries and a [source manifest](results/baseline-2026-10-27/source-manifest.json).
- [An AI assistance and human validation declaration](docs/ai-usage.md), distinguishing assisted implementation/documentation from the tests and simulation operations I performed.

The evidence index also records missing screenshots and the saved-copy discrepancy. The earlier saved CSV and the full export of the supplementary run are two snapshots of one trial, not separate trials.

## Current completion boundary

This version delivers the visual-navigation baseline and its preliminary validation evidence. Camera-input attacks, LiDAR cross-sensor verification, automated collision/clearance and latency evaluation, and the formal attack/defense comparison have not been implemented or validated in this version.

