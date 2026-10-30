# Preliminary baseline evidence from 27 October 2026

These files document manual development trials run by Wu Haonan in the Ubuntu VM. They are not a formal attack/defense evaluation. All 12 timestamped run directories supplied in the export are retained, including observation-only and immediate-stop runs. Successful approaches are not the only outcomes included.

## Provenance

Original run files were exported from `~/baseline-results` through the VMware shared folder. Source CSV and JSON files were copied without content edits. `source-manifest.json` records the relative export source, byte size and SHA-256 of copied evidence. Per-run `summary.json` files and the aggregate [`summary.csv`](summary.csv) were recomputed from the exported original CSV using the included `scripts/summarize_baseline.py`.

Run IDs use UTC; the contributor's desktop timezone was Asia/Shanghai (UTC+8). The source revisions are pinned in `configs/turtlebot3.repos`. Files under `src/`, `configs/`, `scripts/` and `worlds/` were byte-compared with the exported Ubuntu workspace and matched the submission source. The original workspace was an extracted ZIP with no `.git`, so a historical experiment commit cannot be asserted; this submission establishes a Git version of that source.

The exported build command log records return code 0 and an empty stderr file. The 10-test pass is supported by the contributor's pasted Ubuntu terminal transcript, not a separately captured original test-output file. Seven ROS-independent control tests were rerun during submission preparation on Windows; ROS and OpenCV tests were not rerun there. The supplied test transcript is summarized in `docs/ai-usage.md`.

## Main trials

| UTC run ID | Role | Final forward displacement (m) | Final state | Screenshot |
|---|---|---:|---|---|
| 20261027T105705.257055Z | Additional clear-route run found in export | 1.000375 | GOAL_REACHED | Not captured |
| 20261027T110120.535377Z | Reported clear-route validation | 1.002075 | GOAL_REACHED | Not captured |
| 20261027T120711.777818Z | Initial moving obstacle approach | 0.934525 | OBSTACLE_STOP | Not captured |
| 20261027T121506.108892Z | Repeat 1 | 0.933925 | OBSTACLE_STOP | [repeat-1.png](runs/20261027T121506.108892Z/repeat-1.png) |
| 20261027T123745.606716Z | Repeat 2 | 0.934675 | OBSTACLE_STOP | [repeat-2.png](runs/20261027T123745.606716Z/repeat-2.png) |
| 20261027T125230.325259Z | Additional independent trial for complete stopping evidence | 0.944025 | OBSTACLE_STOP | [repeat-3.png](runs/20261027T125230.325259Z/repeat-3.png) |

All six rows have a final logged forward odometry velocity of 0.0 m/s. The four moving obstacle approaches reached a maximum visual ratio of 0.06056134259259259 against a 0.0600 threshold. The obstacle approaches used a 4 m endpoint override, 0.05 m/s commanded forward speed and a red box placed at world X=4 m according to the manual procedure. The clear-route runs used a 1 m endpoint.

The first obstacle approach's terminal report rounded its stop-trigger displacement to 0.934 m; its final CSV displacement is 0.934525 m. Terminal trigger distance and final displacement are different measurements and should not be mixed.

## Other retained development runs

- `20261027T105229.316293Z`: disabled observation, no detected red obstacle.
- `20261027T111529.787456Z` and `20261027T112331.082632Z`: disabled observation with a large red component (about 0.476).
- `20261027T111929.215933Z`: enabled but immediately stopped on the initial image; there was no RUNNING state. This is not a successful moving approach.
- `20261027T113648.513485Z`: disabled observation with ratio about 0.0632, already above the stopping threshold.
- `20261027T120509.371552Z`: disabled observation with ratio about 0.03394 before the moving-approach sequence.

The contributor's informal displayed estimate of 0.02 before one trial differs from the recorded initial ratio near 0.03394; use the logged values for numerical analysis. The screenshot evidence is associated with the explicit run folders supplied by the contributor. It shows the reported stopping scene, not a synchronized ground-truth distance measurement.

## Saved-copy discrepancy

The supplementary trial's early saved CSV contained 1,724 samples and produced the summary pasted in the conversation. The exported original contains 9,191 samples: additional samples were logged after the stopping event. The early CSV was verified to be an exact byte prefix of the original. Both end at the same displacement and zero forward velocity. The original full run is under `runs/20261027T125230.325259Z`; the earlier copy and its original summary are retained under `saved-snapshots/supplementary-IHij0T`. These are two snapshots of one trial, not two independent trials.

## Human observations and limitations

The contributor explicitly reported no contact for the initial moving obstacle trial. Exported English `observation.txt` files report no contact and a complete stop for repeat 1, repeat 2 and the supplementary trial. During saving, the repeat-1 observation file was overwritten while the user was trying to save repeat 2; raw CSV files were unaffected. The final supplied repeat-1 observation is retained, but that record-editing history means its per-trial wording is not an independently timestamped observation. Its own CSV and separately associated screenshot remain available.

The first moving-obstacle trial has no screenshot. No before-motion screenshots were found in the export. Later screenshots are not presented as replacements for those missing images. These are manual no-contact observations, not an automated collision detector or minimum-clearance evaluation.

The three screenshot-supported final displacements span approximately 0.0101 m. This small fixed-scene sample is useful for development checks but does not establish robustness, calibrated safety margins or statistically generalizable performance. There was no controlled random-noise study. Formal evaluation still needs matched four-group trials, timing/clearance metrics, repeated variable conditions and retention of failures.

## Recompute a summary

From the repository root:

```bash
python3 scripts/summarize_baseline.py results/baseline-2026-10-27/runs/20261027T125230.325259Z
```

Keep this evidence associated with the submitted source/configuration. Do not treat later parameter tuning as having generated these results.
