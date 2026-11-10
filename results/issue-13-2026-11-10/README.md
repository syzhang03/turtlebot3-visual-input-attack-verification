# Issue 13 evaluation evidence

On 10 November 2026, 40 formal Gazebo trials were collected under the four-group protocol. The eight scene/input/controller conditions each have five independent fresh-scene runs. The initial eight pilot runs and two subsequent instrumentation checks are kept in separate directories.

| Outcome | Camera-only | Camera–LiDAR |
|---|---:|---:|
| 1 s replay attack success | 5/5 | 0/5 |
| Occlusion attack success | 5/5 | 0/5 |
| Normal empty-route task success | 5/5 | 5/5 |
| Normal obstacle task success | 5/5 | 5/5 |
| Premature stops on normal empty routes | 0/5 | 0/5 |
| Confirmed anomaly alerts on normal inputs | N/A | 0/10 |

An attack success here means a proven violation of the 0.35 m minimum clearance. Across both attack types in Camera-only, every trial's sampled minimum clearance upper bound was approximately 0.132 m. Camera–LiDAR confirmed anomalies in 10/10 attacked trials and kept the lowest sampled clearance lower bound above 2.07 m. The two normal-obstacle groups both maintained the 0.35 m margin. These are geometric bounds derived from native simulator poses. No contact-event count is available from this instrumentation.

The mean first-accepted-frame-to-alert interval was 1.955 simulation seconds for replay (5/5 measurable) and 1.924 for occlusion (4/5 measurable). In `D-occlusion-obstacle-1`, the controller had accepted masked frames before its simulation clock synchronized and recorded `0.0` for their acceptance timestamps. Delivery headers and independent evidence still verify the exposure and the safety outcome. Treat its exact first-frame-to-alert latency as unavailable. Its later motion-to-alert interval was 1.889 s.

In the defended attack groups, command-to-observed-braking-onset averaged 0.028 simulation seconds for replay and 0.023 for occlusion. The five undefended replay runs issued a stop command after speed had already fallen, so none supplied a measurable command-triggered braking onset. The five undefended occlusion runs reached the observation horizon without a controller stop command. Their missing braking values are retained as unavailable. The stop command, independently observed braking onset and sustained near-zero speed are separate measurements.

## Evidence and audit

- `formal/REPORT.md` and `formal/metrics.json` contain all required metrics, per-condition denominators, timing dispersion and missing observations.
- `formal/runs.json` and `formal/status.json` are the original collection verdicts. Collection used commit `0aec88b`; all 32 frozen source hashes match that commit. The runner found unchanged hashes at the end of the batch.
- The original collection validator accepted only a publisher-count sequence containing 1. Five trials also included a startup discovery sample of 0, so it initially marked them invalid. `scripts/audit_issue13.py` reanalyzes the raw records, requiring a discovered publisher, no sampled competing publisher, and an actual forward command stream. It recovered those five trials without discarding, replacing or rerunning them. `formal/analysis-status.json` lists all five; all 40 formal attempts are analyzable. The original verdicts and raw files remain intact.
- `pilot/` used the initial forwarding-observer clock and endpoint policy. The supplemental `pilot-instrumentation/` runs verified the controller-side acceptance trace and the empty-route-only endpoint policy before the formal run. The initial masked pilot demonstrated a wheel-odometry endpoint report before the model reached the endpoint.
- The final regression logs are under `tests/`. The first shell-loop launch produced an empty package argument and is preserved in `test-launch-attempt.log`. The successful individual invocations cover 29 visual, 19 LiDAR and 12 monitor tests.

See `docs/issue-13-evaluation.md` for reproduction and operational definitions. This is the matched comparison of the existing controller configurations; Camera-only and fusion retain their native timer periods of 0.10 and 0.05 wall seconds. The results support the defense-on/off comparison in these fixed scenes. An incremental-benefit comparison with LiDAR-only and confirmation-threshold sensitivity are separate evaluations. Codex ran the experiments and prepared the evidence. Independent teammate review is pending.
