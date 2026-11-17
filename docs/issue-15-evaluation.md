# Matched three-controller Gazebo comparison

Issue #15 adds LiDAR-only to the four scene/input conditions used for both controllers in Issue #13. The two attack conditions are separate. The design has five fresh-scene repetitions per controller and condition.

| Condition | Camera-only | LiDAR-only | Camera–LiDAR |
|---|---|---|---|
| Normal empty 4 m route | Reuse 5 | Add 5 | Reuse 5 |
| Normal obstacle approach | Reuse 5 | Add 5 | Reuse 5 |
| 1 s replay with obstacle | Reuse 5 | Add 5 | Reuse 5 |
| Black-rectangle occlusion with obstacle | Reuse 5 | Add 5 | Reuse 5 |

## Compatibility and protocol

The audit checks the 40 historical records against the frozen source blobs at `0aec88b`, the condition counts, protocol fields and recorded per-trial controller parameters. It checks that the LiDAR-only guard matches the guard used inside fusion, including the corridor, scan topic, stop threshold inputs, control period and timeout policy. Fusion uses the calibrated three-pair confirmation setting. All modes use 1.0 m/s nominal speed and the 4 m endpoint stop only on empty routes.

The shared evaluator records one final command stream, native Gazebo model positions and the same geometric clearance interval as Issue #13. The 0.35 m safety threshold and success definitions remain those in `docs/issue-13-evaluation.md`. In LiDAR-only trials, the evaluator verifies the replay or mask in the common image stream during the approach. The LiDAR controller makes decisions from scan and odometry. The offline audit also requires manipulated frames between the first forward command and the first stop command in each LiDAR-only attack trial.

The replay uses the same 1 s setting, and the occlusion overlay uses the teammate's fixed mask at `fa0706a2b460d63d59ffab6db458b6ed3e4fedf6`. Attacks are active before enabling motion. Both batches use the same pinned world defaults. No extra noise or explicit seed is configured. The new LiDAR-only trials form a separate collection batch, so the three controller runs are not interleaved. This limits interpretation of small timing differences.

The common metrics are attack success, normal task success, geometric clearance, and command-to-braking onset and settling. Empty-route task time and premature stops are reported for all three controllers. Empty-task time ends at the onset of sustained near-zero speed after reaching the goal. Visual anomaly detection latency and false anomaly alarms are applicable to fusion, with missing timing values retained explicitly. The pose-based bounds support safety-margin classification at sampled positions. A separate contact-event rate is unavailable.

## Reproduce

In a sourced ROS Jazzy, TurtleBot3, project, and teammate occlusion-overlay environment, use fresh output folders. The audit needs Git history containing the frozen Issue #13 commit.

```bash
python3 scripts/run_issue13.py --lidar-only --pilot --output /absolute/new-pilot-dir
python3 scripts/report_issue15.py results/issue-13-2026-11-10/formal /absolute/new-pilot-dir --pilot --output /absolute/new-report-dir
python3 scripts/run_issue13.py --lidar-only --output /absolute/new-formal-dir
python3 scripts/report_issue15.py results/issue-13-2026-11-10/formal /absolute/new-formal-dir --output /absolute/new-report-dir
```

The runner retains pilot and formal results in separate directories and verifies frozen hashes at the end of each batch. The report retains unsuccessful or indeterminate outcomes with explicit denominators. The existing Issue #13 report and raw records remain the source of the reused data.
