# Reset scene LiDAR only repetitions

Twenty trials were run on 6 November 2026. Each trial starts a new Gazebo scene and a new controller from the origin. Controller configuration is held at 1.0 m/s, 0.35 m minimum clearance, 0.30 s reaction allowance, 1.0 m/s² assumed deceleration, and a 1.15 m trigger threshold at nominal speed. The three obstacle conditions share the same fixed red-box scene. The empty condition uses an actual empty world and a 4 m endpoint.

## Protocol

Run `scripts/run_lidar_repetitions.py --output <new-folder> --repeats 5` in the sourced ROS/TurtleBot3/project/teammate-occlusion environment. Four conditions are rotated in order across repetitions. Each simulator has a distinct Gazebo partition; ROS uses domain 78. The zero-delay and delayed conditions use the same replay relay. Motion begins only after ZERO_DELAY, REPLAY, or a received masked image confirms readiness. The empty-world condition has no camera relay.

No added noise or explicit random seed is configured. These are reset repeated simulation checks, not a controlled noise experiment or a complete three-controller comparison. The unchanged teammate occlusion source is commit `fa0706a2b460d63d59ffab6db458b6ed3e4fedf6`, installed in a separate overlay.

## Results

Values below are mean ± sample standard deviation across five trials. Time is from first RUNNING to the first terminal stop command, not to physical rest.

| Condition | Expected outcome count | Minimum LiDAR gap per run (m) | Time to stop command (simulation s) |
|---|---:|---:|---:|
| Obstacle, zero-delay relay | 5/5 OBSTACLE_STOP | 1.042 ± 0.040 | 3.128 ± 0.035 |
| Obstacle, 1 s replay | 5/5 OBSTACLE_STOP | 1.039 ± 0.049 | 3.128 ± 0.049 |
| Obstacle, black ROI | 5/5 OBSTACLE_STOP | 1.040 ± 0.047 | 3.131 ± 0.046 |
| Empty world, 4 m route | 5/5 GOAL_REACHED | No finite corridor return | 4.529 ± 0.016 |

All final measured forward speeds were zero. The minimum recorded LiDAR corridor gap across the 15 obstacle trials was 0.976133 m, above the configured 0.35 m margin. Empty-world runs had zero OBSTACLE_STOP outcomes out of five; this is an observed count in this scene, not an estimate of general false-alarm performance. Endpoint overshoot averaged 0.032277 m, sample SD 0.016078 m, range 0.012277–0.053277 m.

Wall-clock stop-command times are stored separately in `statistics.json` and must not be mixed with simulation durations. All twenty attempts and their exit codes are retained in `runs.json`; no failed attempts were discarded. Per-trial folders retain raw controller CSV/config, attack logs and simulator logs. `summary.json` stores counts; `statistics.json` can be regenerated with `scripts/summarize_lidar_repetitions.py`.

## Limits and provenance

The gaps come from the controller's LiDAR projection to a configured envelope. No independent simulator pose/contact measurement was collected. Collision rate remains unmeasured, explicitly null in the statistics. These data support operation of the LiDAR-only baseline under the tested conditions, not a claim that fusion improves safety or efficiency.

The source manifest records the local source snapshot when the batch started. A subsequent runner-only change requests graceful verifier cleanup on watchdog timeout or keyboard interruption; that branch was not exercised by these successful trials. No LiDAR controller parameters or logic changed during the batch.

Codex authored the scripts and ran the automated checks. No independent teammate review is claimed. No GitHub issue was closed or PR merged by this experiment.
