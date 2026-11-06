# Frozen Camera LiDAR validation results

Date: 7 November 2026. The calibrated monitor and controller parameters were held fixed while 53 fresh-scene trials were executed. All 53 achieved their predeclared outcomes and sampled simulator-pose margin requirements. No unsuccessful scene attempt was removed or replaced.

## Results

| Condition | Trials | Observed result |
|---|---:|---|
| Normal obstacle approach | 5 | 5 normal obstacle stops; no anomaly alerts |
| Obstacle with 1 s camera replay | 5 | 5 VISUAL_MISS confirmations and anomaly stops |
| Obstacle with 0.5 s replay | 5 | 5 VISUAL_MISS confirmations and anomaly stops |
| Obstacle with black ROI | 5 | 5 VISUAL_MISS confirmations and anomaly stops |
| Empty 4 m route, normal images | 5 | 5 completed; no anomaly alerts |
| Empty 4 m route, 1 s replay | 5 | 5 completed; no anomaly alerts |
| Empty 4 m route, black ROI | 5 | 5 completed; no anomaly alerts |
| Early camera loss, scan loss, image-stamp freeze | 9 | 3/3 per fault stopped on SENSOR_TIMEOUT |
| Near-obstacle camera loss and image freeze | 6 | 3/3 per fault stopped first on the LiDAR emergency guard |
| Near-obstacle scan loss | 3 | 3/3 stopped on SENSOR_TIMEOUT |

The smallest independent pose-derived clearance lower bound across all non-empty trials was **0.748587 m**, in a near-obstacle scan-loss run, above the fixed 0.35 m minimum. All final measured forward speeds were below 0.02 m/s. Actual values are zero or numerical roundoff.

The twenty trials that should not produce a cross-sensor alarm (normal obstacle approaches plus all empty routes) produced none. The fifteen attacked obstacle approaches all confirmed discrepancies. These are finite counts in these conditions, not a general false-positive probability or an attack detection guarantee.

Raw pairing audits passed for all 53 trials: each image stamp and each scan stamp appears in at most one pair, all pairs satisfy the 0.12 s tolerance, ambiguous pairs do not accumulate evidence, and first alerts occur at count three. The fifteen confirmed sequences each span 0.2 simulation seconds from their first pair to their confirming pair. This is a confirmation interval, not attack-onset detection latency.

## Two-phase protocol and provenance

1. `frozen-campaign/`: 44 attempts, source and parameter hashes unchanged throughout. The six main route/attack conditions and the short-delay condition have five repetitions each. The three early faults have three repetitions each. Outcome choices, including safety-only allowance for short replay, were written before the runs. Actual short-replay runs all detected the discrepancy.
2. `late-faults/`: nine further attempts, with a fault injected when the logged LiDAR corridor gap first reaches 1.5 m. All three faults have three repetitions. The predeclared accepted states are SENSOR_TIMEOUT or OBSTACLE_STOP: an active emergency guard is allowed to beat the camera watchdog. The requested fault must actually be injected, not merely requested.

Each phase has its own protocol/hash manifest and confirms its sources unchanged during that phase. Between phases only the experiment harness gained a configurable fault-injection distance and explicit injection verification. The calibrated controller source and YAML were not changed. The first phase's exact harness is retained in Git commit 87e3587. Calibration remains the separate clean run from 6 November; no evaluation data was used to retune thresholds.

Per-run folders retain raw scene, relay and controller logs, config snapshots, pair decisions, native simulator poses and summary. Every scene is newly started at the origin; condition order rotates during the main phase, and each scene uses an isolated Gazebo partition. There is no additional noise injection or explicitly configured random seed, so this is not a controlled noise study.

Camera fault injection uses a common image-forwarding hop present in every fusion trial. Camera freeze repeats the last full frame including its old timestamp; the separate replay attack supplies old contents under current timestamps. These exercise different defenses.

## Reports and reproduction

- Main outcomes and statistics: `frozen-campaign/REPORT.md`, `summary.json`, `runs.json`, `stability-audit.json`.
- Main pairing audit: `frozen-campaign/pair-audit.json`.
- Near-obstacle raw outcomes: `late-faults/runs.json`, `summary.json`, `pair-audit.json`.
- Final regression: `tests/monitor.log` (12), `tests/lidar.log` (19), `tests/visual.log` (29), all passing.

After sourcing ROS, TurtleBot3, the project install and teammate occlusion overlay, use a new output directory:

```bash
python3 scripts/run_fusion_campaign.py --output /absolute/new-main-folder
python3 scripts/report_fusion_campaign.py /absolute/new-main-folder
python3 scripts/audit_fusion_pairs.py /absolute/new-main-folder
python3 scripts/run_fusion_late_faults.py --output /absolute/new-late-folder
python3 scripts/audit_fusion_pairs.py /absolute/new-late-folder
```

## Conclusion and scope

The implementation shows repeatable intended behavior for this calibrated centered red obstacle, robot, sensor arrangement, speed and tested image manipulations. Camera loss/freeze near an obstacle correctly preserves LiDAR emergency priority. The test batch supports proceeding to code review; it does not automatically approve or merge a PR.

Pose-based bounds use a verified 0.30 m robot enclosure and known red-box footprint. They are independent of the controller's LiDAR and odometry but remain sampled geometric bounds, not a contact sensor or exact mesh distance. A matched comparison against Camera-only and LiDAR-only is still required to establish any extra benefit. These outcomes do not establish general fusion superiority or robustness to other obstacles, scenes, noise, attacks or LiDAR compromise.

Codex authored the harness extensions, ran the experiments, audited the logs and prepared this report. Independent teammate review has not been claimed. No merge was performed during this validation task.
