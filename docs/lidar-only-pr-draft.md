# Add a LiDAR only comparator for obstacle avoidance

Closes #7

## Changes

Add a standalone ROS 2 LiDAR-only controller so the team can compare Camera–LiDAR verification with a trusted single-sensor baseline. The controller transforms scans into the robot frame, filters a forward corridor, and stops for obstacles, stale/unusable sensors, endpoint completion, or command-publisher conflicts. It records configuration, decisions, commands and odometry.

Add configurable launch/build helpers, geometry and real DDS tests, and a Gazebo development verifier that can run with the existing delay relay or a separately installed copy of the teammate's occlusion package. Existing visual controller and attack sources are unchanged.

## Validation

- Two project packages built in WSL Ubuntu 24.04 / ROS Jazzy.
- Existing visual package: 29 tests passed.
- New LiDAR package: 19 tests passed, including 5 real DDS tests and 3 evaluation-geometry checks.
- Teammate occlusion overlay: 18 tests passed, including 6 real DDS tests.
- Gazebo: clear segment, LiDAR interruption and obstacle approach passed without camera manipulation, with 1 s camera replay, and with black-rectangle camera occlusion.
- Separate empty-world endpoint check passed.
- Twenty reset-scene repetitions: 5/5 expected outcomes each for clean, delayed and occluded obstacle approaches and the empty-world 4 m task. The smallest sensor-derived corridor gap was 0.976133 m; ground-truth collision rate remains unmeasured.
- Three extra native Gazebo pose checks: conservative robot–obstacle clearance lower bounds 0.924436 m clean, 0.926256 m replay, 0.940109 m occlusion. Evaluation geometry is independent of control sensors. The failed initial bridge attempt is retained.
- Acceptance map: `docs/issue-7-acceptance.md`.
- Evidence: `results/lidar-only-2026-11-05` and `results/lidar-only-2026-11-06`.

## Limitations and review

The earlier three-case suites are sequential development checks, supplemented by twenty reset trials and three simulator-pose clearance checks. Pose checks use a conservative robot enclosure, not a contact sensor. Braking parameters need calibration before formal comparison. Camera–LiDAR fusion and path planning are outside this change.

AI assistance: Codex implemented the new baseline and verification harness and ran the recorded checks. The camera-occlusion implementation is the teammate's unchanged code. No independent human verification has yet been recorded. Keep this PR in draft until a teammate reviews the implementation, reproduces the relevant checks, and resolves the remaining acceptance evidence for #7.
