# Issue 13 four-group evaluation

Recorded 40 of 40 planned attempts. 40 yielded analyzable evidence. Frozen source check: True.

| Group / input / scene | Attack success | Normal task success | False alarms | Premature stops |
|---|---:|---:|---:|---:|
| A / clean / empty | N/A | 5/5 | N/A | 0/5 |
| C / clean / empty | N/A | 5/5 | 0/5 | 0/5 |
| A / clean / obstacle | N/A | 5/5 | N/A | N/A |
| C / clean / obstacle | N/A | 5/5 | 0/5 | N/A |
| B / replay / obstacle | 5/5 | N/A | N/A | N/A |
| D / replay / obstacle | 0/5 | N/A | N/A | N/A |
| B / occlusion / obstacle | 5/5 | N/A | N/A | N/A |
| D / occlusion / obstacle | 0/5 | N/A | N/A | N/A |

| Group / input / scene | Clearance bound range across runs (m) | Mean detection (sim s) | Mean braking onset (sim s) | Mean settling (sim s) |
|---|---:|---:|---:|---:|
| A / clean / empty | N/A | N/A | 0.024 (5/5 measured) | 0.123 (5/5 measured) |
| C / clean / empty | N/A | N/A | 0.022 (5/5 measured) | 0.120 (5/5 measured) |
| A / clean / obstacle | [0.507, 0.950] | N/A | 0.022 (5/5 measured) | 0.124 (5/5 measured) |
| C / clean / obstacle | [0.832, 1.293] | N/A | 0.025 (5/5 measured) | 0.121 (5/5 measured) |
| B / replay / obstacle | [-0.232, 0.132] | N/A | N/A | 0.003 (5/5 measured) |
| D / replay / obstacle | [2.071, 2.459] | 1.955 (5/5 measured) | 0.028 (5/5 measured) | 0.126 (5/5 measured) |
| B / occlusion / obstacle | [-0.232, 0.132] | N/A | N/A | N/A |
| D / occlusion / obstacle | [2.092, 2.538] | 1.924 (4/5 measured) | 0.023 (5/5 measured) | 0.120 (5/5 measured) |

See `metrics.json` for separate lower/upper-bound statistics and per-condition timing ranges. A clearance interval reflects uncertainty in sampled collision geometry. A threshold violation is confirmed only when the upper bound is below 0.35 m. The lower bound establishes a safe margin when it reaches 0.35 m. The contact event rate is unavailable because these runs have no contact sensor.

Detection uses the first accepted manipulated frame and the confirmed alert on simulation time. One occlusion trial accepted its first manipulated frame before its controller clock synchronized; its detection latency is unavailable. Braking onset uses sustained independently measured speed decrease after the stop command. Trials where motion slowed before the command or no stop command occurred have unavailable braking latency. See `analysis-runs.json` (or `runs.json` before audit) for those reasons and any indeterminate outcomes.
