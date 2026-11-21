# Evaluation reproduction

## Scope and attribution

This package performs secondary analysis of 200 previously collected reliability trials. It does not collect new data, implement a new defense, or replace the team's existing experiment runners. The copied source CSV, protocol and report are retained unchanged under `data/reliability-main-2026-11-18/` for standalone reproduction. Their original authorship remains with the team.

The upstream source is the `results/reliability-main-2026-11-18/` evaluation documented at commit `af50cb7` in https://github.com/syzhang03/turtlebot3-visual-input-attack-verification . The source report identifies collection controller `aef5380` and a local reliability harness. This analysis does not establish that the entire reliability harness is available upstream.

## Run

From this package root, with Python 3.10 or newer:

```bash
python -m pip install -r requirements.txt
python scripts/analyze_reliability_results.py
```

Or pass explicit paths:

```bash
python scripts/analyze_reliability_results.py --input data/reliability-main-2026-11-18/trials.csv --output results/final-analysis
```

Paths with spaces must be quoted. The default input and output are relative to the script location, so the default command also works from another directory. No ROS, Gazebo, GPU or network connection is required after matplotlib is installed.

Rerunning replaces generated analysis files in the selected output directory. The input CSV is not changed. `provenance.json` records the input and script SHA-256 hashes and actual Python/matplotlib versions. Fonts and SVG metadata can differ across environments even when the numerical results match.

## Files generated

| File | Meaning |
|---|---|
| `summary.csv` | 20 scene/input/noise cells, each with 10 trials |
| `by_profile.csv` | Five input profiles, each with 40 trials |
| `overall.csv` | Aggregate descriptive statistics; scene-conditioned fields are identified by name |
| `noise_pairs.csv` | 100 matched noise-off/on records with exposure flags |
| `validation.json` | Matrix, ID and supplied-group-count checks |
| `provenance.json` | Source hashes and analysis environment |
| `REPORT.md` | Results, interpretation and limitations |
| `figures/*.png` and `figures/*.svg` | Four plots, in raster and editable vector formats |

## Metric definitions

| Field | Definition and denominator |
|---|---|
| `normal_task_success_rate` | True flags / nonmissing task judgments. Empty scene: complete the 4 m route. Obstacle scene: expected normal obstacle stop plus verified clearance. |
| `false_stop_rate` | True original false-stop flags / nonmissing false-stop judgments. Protective stops are not automatically false stops. |
| `restricted_rate` | Trials entering restricted state / trials with a recorded restricted flag. Not a detection rate. |
| `noise_exposed_n` | Confirmed exposure among trials assigned noise on. Scheduled but unexposed trials remain in their assigned group. |
| `clearance_lower_m_*` | Statistics of the per-trial minimum independent geometric lower bound in obstacle trials only. Empty-scene clearance is unavailable, not zero. |
| `successful_empty_completion_sim_s_*` | Motion-to-stop-command interval + command-to-settling interval, only for successful empty-route trials with both measurements. |
| `motion_to_stop_command_s_*` | Motion start to first stop command; not complete physical braking. |
| `braking_latency_s_*` | Existing recorded braking-onset latency; use original measurement definition. |
| `settling_latency_s_*` | Existing command-to-settling interval. |
| `detection_latency_s_*` | Existing detection latency where available. Missing entries are not replaced with zero. |

Every numerical summary reports its own `n`. SD is the sample SD (n−1 denominator); it is blank when fewer than two observations exist. All times are simulation seconds. Percentages in the plots are descriptive proportions with count labels, not population estimates.

## Validation and interpretation

The script rejects missing required columns, duplicate trial IDs, invalid trials, incomplete 20×10 matrices and malformed noise pairs. It compares group trial counts, task successes, false stops, restricted entries and alert-trial counts with `group-metrics.csv` when that file is beside the input CSV. It retains original scoring and all assigned trials.

The source report identifies eight timing-boundary false stops, but the summary CSV does not contain scan-age traces sufficient to independently reproduce that finding. This package explicitly attributes that statement to the source report. Access the team's raw trial evidence before changing those labels.

No matched old-policy comparison is present. No direct contact-event logging was collected. Do not call the geometric margin result a measured zero-collision rate or a demonstrated improvement over the old policy.

## Integration into the team repository

The team already has `scripts/run_issue13.py`, `scripts/report_issue15.py` and related runners/reporters. Keep those files. Add this analysis script, this documentation and generated results as a focused PR.

The copied `data/` directory is a convenience for this standalone delivery. Before merging, point the script at the existing canonical `trials.csv` in the repository and avoid committing a duplicate dataset if the same bytes already exist. The optional `--input` argument supports this without modifying the script. Preserve an explicit source path and hash in the PR description.

Review the calculations and rerun the command yourself. Describe the personal contribution as reviewed secondary analysis, validation and documentation. Claim new experiment automation or data collection only if you actually complete those additional tasks.
