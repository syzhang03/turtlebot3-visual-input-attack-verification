# Four matched Gazebo evaluation groups

Issue #13 evaluates Camera-only (defense off) and Camera–LiDAR (defense on) using the proposal's four groups. The fusion controller uses its frozen three-pair confirmation configuration. Threshold sensitivity and the separate LiDAR-only comparison belong to separate analyses.

| Group | Control | Input and scene |
|---|---|---|
| A | Camera-only | Clean empty route and clean obstacle approach |
| B | Camera-only | Obstacle approach with 1 s replay or fixed black-region occlusion |
| C | Camera–LiDAR | Clean empty route and clean obstacle approach |
| D | Camera–LiDAR | Obstacle approach with 1 s replay or fixed black-region occlusion |

## Matched protocol

Run the eight specific conditions five times each. Each trial launches a fresh Gazebo scene at the same initial robot pose with an isolated transport partition. Matching trials use the same world, red obstacle when applicable, nominal 1.0 m/s speed, visual detector and region of interest, 0.35 m safety margin, and camera manipulation. The existing zero-delay relay is the clean input. Camera replay/masking starts before motion and its delivery is verified. The world and noise settings are shared and no additional seed or noise injection is configured, so exact random sequences are not asserted. Rotate condition blocks between repeats. The runner hashes the controller, scenario, parameters and harness before the batch and checks those hashes afterward.

Only empty routes use the odometry endpoint stop. Both controller modes use a 4 m goal in that scene. In the obstacle scene the experiment ends at a terminal controller state plus a settling observation period, or after an eight-simulation-second observation horizon measured from the first observed RUNNING record. At the horizon the harness shuts down the controller for cleanup. Shutdown zero commands are outside the measurement interval. In the initial pilot, odometry reported the endpoint during the masked obstacle approach even though the Gazebo model remained short of it. This informed the symmetric empty-route-only endpoint policy. The pilot remains separate from the formal results.

Each trial records its own configuration, camera relay logs, controller events, accepted image stamps, commands, alert events and native simulator poses. The harness observes `/cmd_vel` and checks publisher ownership. Neither controller consumes evaluator ground truth. The existing launch and evaluation entry points remain available; the added evaluation mode records adverse outcomes instead of requiring a successful safety stop.

## Measurement rules

- **Attack success** means a collision or minimum clearance below 0.35 m. For this fixed box, a sampled clearance interval is calculated independently from native Gazebo robot pose. An enclosing 0.30 m sphere gives a conservative lower bound. Distance from a known interior point of the pinned robot base collision box to the red box gives an upper bound. A lower bound at or above 0.35 m establishes safety at the sampled poses. An upper bound below 0.35 m establishes a violation. A straddling interval is indeterminate and remains in the denominator. These trials do not have direct contact sensing; report the proven threshold violations rather than an inferred collision count.
- **Normal task success** requires absolute cross-track error at most 0.15 m. On an empty route, the controller must report goal completion, the independent model pose must reach at least 3.98 m (2 cm endpoint tolerance), and the robot must settle. In a normal obstacle approach, the controller must issue an obstacle stop, settle and maintain the sampled safety margin. Count premature stops on the empty route separately from diagnostic false alarms.
- **Detection latency** uses the simulation timestamp when the controller accepted the first manipulated frame and the timestamp when it published the first confirmed anomaly. Match accepted image header stamps against the relay replay rows or the confirmed black mask in the evaluator's delivered stream. Trace accepted images only when this experiment enables it. A disabled detector, absent alert, or first acceptance before controller clock synchronization yields no latency measurement.
- **Actual braking latency** starts at the first recorded zero forward command after forward motion. Estimate speed independently from longitudinal native pose change over a trailing window of approximately 0.10 s. Braking onset is the start of a speed drop at least 0.10 m/s below the pre-command reference that persists for 0.10 s. Settling requires absolute measured speed at most 0.02 m/s for 0.15 s. Sustained segments are broken by sample gaps over 0.10 s. A robot already slowed before the command, a missing command or insufficient samples yields an unavailable braking-onset value. These are sampled, smoothed timing estimates.
- **False alarm rate** counts clean fusion trials with any confirmed anomaly alert. Camera-only has no anomaly detector. Normal obstacle stopping is counted as task success when the safety and settling criteria hold.

## Reproduce

Source ROS Jazzy, the pinned TurtleBot3 and project builds, and the separately installed teammate occlusion package. The occlusion overlay in this deployment was built from `fa0706a2b460d63d59ffab6db458b6ed3e4fedf6`. Use a fresh output directory and no other controllers on the evaluation ROS domain.

```bash
python3 scripts/run_issue13.py --pilot --output /absolute/new-pilot-dir
python3 scripts/run_issue13.py --output /absolute/new-formal-dir
python3 scripts/audit_issue13.py /absolute/new-formal-dir
python3 scripts/report_issue13.py /absolute/new-formal-dir
```

The formal output contains protocol hashes, all trial outcomes and raw evidence, JSON metrics with denominators and missing values, and the Markdown report. The raw historical verifier summaries include a legacy `all_passed` field for the single-controller development checks. Current evaluation summaries mark that field unavailable. `runs.json` records collection validity separately from task success and attack success. The offline audit writes separate `analysis-runs.json` and `analysis-status.json` files. It accepts initial zero-publisher discovery samples when one publisher is subsequently observed, no competing publisher is seen and real forward commands are recorded. The original collection files are retained.

The completed batch and its interpretation are in `results/issue-13-2026-11-10/README.md`.
