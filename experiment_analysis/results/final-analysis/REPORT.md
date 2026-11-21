# Reliability evaluation: independent reanalysis

This submission reanalyzes the team-supplied frozen dataset. It does not claim authorship of the original controller, experiment harness or data collection. No new ROS/Gazebo trials were run.

## Dataset and validation

200 trials; 20 scene × input × noise groups, 10 repeats each. All trial IDs are unique and all records are marked valid. The five grouped counts were checked against the supplied group-metrics.csv; see validation.json.

## Recomputed results

| Input | Task success (both scenes) | False stops | Restricted |
|---|---:|---:|---:|
| Normal | 38/40 | 1/40 | 25/40 |
| Camera delay | 40/40 | 0/40 | 40/40 |
| Camera visor | 19/40 | 19/40 | 24/40 |
| LiDAR loss | 0/40 | 1/40 | 16/40 |
| Dual loss | 0/40 | 7/40 | 40/40 |

All 100 obstacle trials have recorded minimum geometry-based clearance lower bounds above 0.35 m. The minimum is 0.747 m. This is not a direct contact-event measurement.

False stops: 28/200. Noise off: 12/100; noise on: 16/100. There are 7 pairs with a false stop only under noise and 3 only without noise. These are descriptive paired results, not a causal effect or significance claim.

Confirmed noise exposure: 99/100 scheduled noise trials. The unexposed assigned trial is retained in its original group.

### Successful empty-route completion times

| Input | Successful trials with time | Mean ± sample SD (sim s) | Median (sim s) |
|---|---:|---:|---:|
| Normal | 20 | 5.278 ± 0.605 | 5.140 |
| Camera delay | 20 | 5.574 ± 0.410 | 5.402 |
| Camera visor | 19 | 5.311 ± 0.600 | 5.252 |
| LiDAR loss | 0 | N/A | N/A |
| Dual loss | 0 | N/A | N/A |

## Interpretation and limits

- All trials used the new three-state policy; there is no matched old-policy arm. Do not interpret this dataset as a measured improvement over the old controller.
- Task success has scene-specific meaning: finish the empty 4 m route, or make the expected normal obstacle stop with verified clearance. A protective stop may be safe but not task-successful.
- The supplied REPORT.md identifies 8 of the 28 false-stop flags within 1 ms of the 0.5 s scan-age threshold. This CSV lacks scan-age evidence to independently identify those trials. Preserve the original flags; audit the raw evidence before rescoring.
- The source report assigns 19 false stops to the 20 obstacle-visor trials. This differs from the denominator of all 40 visor trials.
- Restricted-state entry is not validated fault detection or sensor attribution. Even nominal input enters restricted mode.
- Completion times exclude failed routes. Compare them alongside task-success rates, not as an unconditional speed ranking.
- Timing is in simulation seconds. Command timing, braking onset and settling are distinct measurements. Missing metrics are not zero.
- Fixed scenes, repeated runs and uncontrolled scheduling limit generalization. The graphs are descriptive; no inferential independence assumption is made.

## Sources

- Supplied Defense Presentation Handoff/Evidence: trials.csv, group-metrics.csv, protocol.json, REPORT.md.
- https://github.com/syzhang03/turtlebot3-visual-input-attack-verification/blob/af50cb7/results/reliability-main-2026-11-18/REPORT.md
- Original collection controller: aef5380, as stated in the supplied report; the integrated branch was validated separately.

## Contribution and AI disclosure

This package contributes a secondary-analysis script, reproduced descriptive summaries, figures, consistency checks and reproduction documentation. ChatGPT/Codex assisted implementation and drafting. Yang Mingzhi should review and understand the calculations before claiming the reviewed analysis as a personal contribution. Original team data and code remain attributed to their authors.
