# Issue 15 three-controller comparison

40 audited Issue #13 records and 20 new LiDAR-only records. 60/60 analyzable trials.

| Condition | Controller | Attack success | Normal task success | Premature stops | False anomaly alarms | Mean empty task completion (sim s) |
|---|---|---:|---:|---:|---:|---:|
| Clean empty route | Camera-only | N/A | 5/5 | 0/5 | N/A | 4.661 (5/5 measured) |
| Clean empty route | LiDAR-only | N/A | 5/5 | 0/5 | N/A | 4.665 (5/5 measured) |
| Clean empty route | Camera–LiDAR | N/A | 5/5 | 0/5 | 0/5 | 4.644 (5/5 measured) |
| Clean obstacle | Camera-only | N/A | 5/5 | N/A | N/A | N/A |
| Clean obstacle | LiDAR-only | N/A | 5/5 | N/A | N/A | N/A |
| Clean obstacle | Camera–LiDAR | N/A | 5/5 | N/A | 0/5 | N/A |
| 1 s replay, obstacle | Camera-only | 5/5 | N/A | N/A | N/A | N/A |
| 1 s replay, obstacle | LiDAR-only | 0/5 | N/A | N/A | N/A | N/A |
| 1 s replay, obstacle | Camera–LiDAR | 0/5 | N/A | N/A | N/A | N/A |
| Occlusion, obstacle | Camera-only | 5/5 | N/A | N/A | N/A | N/A |
| Occlusion, obstacle | LiDAR-only | 0/5 | N/A | N/A | N/A | N/A |
| Occlusion, obstacle | Camera–LiDAR | 0/5 | N/A | N/A | N/A | N/A |

| Condition | Controller | Clearance bounds across runs (m) | Mean start to stop command (sim s) | Mean detection (sim s) | Mean braking onset (sim s) | Mean settling (sim s) |
|---|---|---:|---:|---:|---:|---:|
| Clean empty route | Camera-only | N/A | 4.538 (5/5 measured) | N/A | 0.024 (5/5 measured) | 0.123 (5/5 measured) |
| Clean empty route | LiDAR-only | N/A | 4.540 (5/5 measured) | N/A | 0.021 (5/5 measured) | 0.125 (5/5 measured) |
| Clean empty route | Camera–LiDAR | N/A | 4.524 (5/5 measured) | N/A | 0.022 (5/5 measured) | 0.120 (5/5 measured) |
| Clean obstacle | Camera-only | [0.507, 0.950] | 3.486 (5/5 measured) | N/A | 0.022 (5/5 measured) | 0.124 (5/5 measured) |
| Clean obstacle | LiDAR-only | [0.849, 1.324] | 3.135 (5/5 measured) | N/A | 0.026 (5/5 measured) | 0.123 (5/5 measured) |
| Clean obstacle | Camera–LiDAR | [0.832, 1.293] | 3.141 (5/5 measured) | N/A | 0.025 (5/5 measured) | 0.121 (5/5 measured) |
| 1 s replay, obstacle | Camera-only | [-0.232, 0.132] | 4.510 (5/5 measured) | N/A | N/A | 0.003 (5/5 measured) |
| 1 s replay, obstacle | LiDAR-only | [0.855, 1.316] | 3.120 (5/5 measured) | N/A | 0.024 (5/5 measured) | 0.125 (5/5 measured) |
| 1 s replay, obstacle | Camera–LiDAR | [2.071, 2.459] | 1.952 (5/5 measured) | 1.955 (5/5 measured) | 0.028 (5/5 measured) | 0.126 (5/5 measured) |
| Occlusion, obstacle | Camera-only | [-0.232, 0.132] | N/A | N/A | N/A | N/A |
| Occlusion, obstacle | LiDAR-only | [0.886, 1.309] | 3.115 (5/5 measured) | N/A | 0.020 (5/5 measured) | 0.119 (5/5 measured) |
| Occlusion, obstacle | Camera–LiDAR | [2.092, 2.538] | 1.907 (5/5 measured) | 1.924 (4/5 measured) | 0.023 (5/5 measured) | 0.120 (5/5 measured) |

Clearance entries span the lowest sampled lower bound and highest sampled upper bound of the per-run minima. The data support geometry-based threshold classification. Direct contact events were not recorded.

Empty-task completion time runs from motion start to the onset of sustained near-zero speed at the goal. The start-to-command and command-to-settling intervals are also shown separately. Visual alert timing is applicable to the fusion monitor. Reasons for unavailable measurements and timing ranges are in `metrics.json` and the per-trial records.

The two collection batches use the same protocol and source-verified controller configuration. The design was not contemporaneously interleaved across all three controllers. Exact seeds were not controlled. Interpret results within these fixed scenes and five repeats per cell.
