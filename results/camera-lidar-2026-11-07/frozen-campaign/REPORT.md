# Frozen Camera LiDAR validation campaign

Completed 44/44 predeclared attempts. Frozen-source check: True.

| Condition | Expected outcome | Anomaly alerts | Min pose clearance bound (m) | Mean stop time (sim s) |
|---|---:|---:|---:|---:|
| obstacle-clean | 5/5 | 0/5 | 0.917 | 3.111 |
| obstacle-delay-1 | 5/5 | 5/5 | 2.084 | 1.940 |
| obstacle-occlusion | 5/5 | 5/5 | 2.115 | 1.888 |
| empty-clean | 5/5 | 0/5 | N/A (empty world) | 4.541 |
| empty-delay-1 | 5/5 | 0/5 | N/A (empty world) | 4.526 |
| empty-occlusion | 5/5 | 0/5 | N/A (empty world) | 4.540 |
| obstacle-delay-0.5 | 5/5 | 5/5 | 2.094 | 1.926 |
| camera-loss | 3/3 | 0/3 | 3.415 | 0.502 |
| scan-loss | 3/3 | 0/3 | 3.425 | 0.452 |
| camera-freeze | 3/3 | 0/3 | 3.410 | 0.490 |

## Stability audit

- PASS: all_attempts_retained
- PASS: all_predeclared_trial_outcomes
- PASS: frozen_source_and_parameters
- PASS: final_speed_below_0.02_m_s
- PASS: minimum_pose_bound_0.35_m
- PASS: normal_inputs_no_anomaly_alerts
- PASS: one_second_and_mask_detected
- PASS: all_empty_tasks_reached_goal
- PASS: all_faults_stopped_on_timeout
- PASS: fault_injection_recorded

## Interpretation

The 0.5 s replay condition permits either anomaly stopping or normal LiDAR stopping. Its alert count is reported separately from safe-stop success; missed anomaly confirmation is not hidden by the safety metric.
Normal and empty-task alarm counts are counts over this finite campaign, not a general false-positive probability. Empty attacked routes intentionally contain no obstacle, so camera masking need not imply cross-sensor disagreement.
Fault injection occurs only after RUNNING. Camera freeze repeats the last complete image and its timestamp; it is distinct from the replay attack that rewrites current headers.
Clearance is a conservative bound from sampled native simulator poses and a verified robot enclosure. It is independent of LiDAR/odometry but not a contact sensor or exact mesh distance.
No noise is added and no explicit random seed is configured. Natural execution jitter is not a controlled noise study. A separate multi-controller evaluation is needed to assess fusion benefit.
This report does not approve or merge a PR. Independent teammate review remains separate from agent-run experiments.
