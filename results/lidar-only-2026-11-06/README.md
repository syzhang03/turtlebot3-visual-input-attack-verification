# Occlusion integration development evidence

Date: 6 November 2026. This extends the LiDAR-only development evidence from 5 November. These are development checks, not a formal repeated attack evaluation.

## Sources and setup

- LiDAR source: local `defense/lidar-only` work based on main `d57fff4a77a56f4dcbe11ed37cfc870f935ecb33`.
- Occlusion source: teammate branch `hou_meiqi_camera_attack`, commit `fa0706a2b460d63d59ffab6db458b6ed3e4fedf6`.
- Only `src/tb3_camera_attack` was exported from that commit to `/home/michael/tb3_occlusion_validation`, built and sourced as a separate overlay. No teammate code was changed and no branch was merged.
- Runtime: WSL Ubuntu 24.04 / ROS Jazzy / Gazebo Harmonic with the pinned TurtleBot3 dependency workspace.
- Domain: `ROS_DOMAIN_ID=76`; Gazebo partition: `tb3-lidar-validation-20261106`.
- The suite uses the existing 1.0 m/s LiDAR configuration. It does not reuse the prior 0.3 m/s desktop demonstration.

## Execution

After sourcing ROS, TurtleBot3, the project install and the separate occlusion install:

```bash
ROS_DOMAIN_ID=76 GZ_PARTITION=tb3-lidar-validation-20261106 \
  python3 scripts/verify_lidar_gazebo.py --occlusion --output /absolute/new-output-folder
```

The verifier runs the original `camera_occlusion` executable with `attack_enabled=true` and output remapped to `/attack/image_raw`. The default normalized mask is x=0.2, y=0.25, width=0.6, height=0.75. Its received BGR8 image pixels are checked, not just its startup parameters.

## Results

- Received 319 attacked images; all 319 had an exactly black detector ROI.
- Clear 0.5 m segment: GOAL_REACHED, final relative displacement 0.551276 m.
- LiDAR stream interruption with continuing odometry: SENSOR_TIMEOUT, final relative displacement 0.120989 m.
- Obstacle approach: OBSTACLE_STOP, final relative displacement 1.914276 m.
- All three final forward/angular commands were zero, and final odometry forward speed was zero.
- Final obstacle-run LiDAR corridor clearance: 1.068994 m. Configured trigger threshold: 1.15 m.
- See `occlusion-on/summary.json`, process logs, and per-run raw CSV/config files.

## Teammate package tests

The original occlusion branch had recorded skipped real ROS tests on Windows. They were run here against real ROS packages and DDS:

```bash
ROS_DOMAIN_ID=77 python3 -m pytest \
  ~/tb3_occlusion_validation/src/tb3_camera_attack/test -v -ra
```

18 passed, no skips, including all 6 real ROS relay tests. See `occlusion-package-tests.log`. This is agent-executed validation, not independent teammate review.

## Limits

The three checks run sequentially within one scene. Their origins differ; do not treat their displacements as world coordinates. The clear segment ends before the distant obstacle. There is no independent contact sensor or simulator ground-truth clearance measurement. The reported gap is sensor-derived and the threshold is not the final braking distance.

These runs demonstrate operation while a real camera-occlusion relay is active. They do not establish an attack success rate, matched statistical equivalence, formal false-alarm rate or multi-sensor benefit. LiDAR-only intentionally has no camera input. Camera–LiDAR monitoring remains unimplemented.

## Reset repetitions added

The subsequent `reset-trials/` batch contains twenty independently restarted scene trials: five zero-delay obstacle approaches, five 1 s delay approaches, five occlusion approaches, and five empty-world 4 m tasks. All expected outcomes were observed. See `reset-trials/README.md` for statistics, endpoint overshoot, protocol and limitations. This supplements rather than replaces the earlier sequential checks.

## Independent pose checks added

`ground-truth-clean`, `ground-truth-delay`, and `ground-truth-occlusion-native` contain additional simulator-pose checks independent of LiDAR and odometry. Conservative separation lower bounds were 0.924436, 0.926256 and 0.940109 m respectively. All reached OBSTACLE_STOP with final measured speed zero. See `docs/issue-7-acceptance.md` for geometry and scope.

The initial `ground-truth-occlusion` ROS-bridge attempt failed before movement because no expected named poses were received. It remains in the evidence and is not counted as passing. The evaluator was corrected to use native Gazebo messages, retaining model names. `final-lidar-tests.log` records all 19 current LiDAR tests passing, with no skips.

AI assistance: Codex extended the verification harness, ran the checks, and prepared this record. The original occlusion implementation remains the teammate's contribution. Independent human review and formal experiment design remain pending.
