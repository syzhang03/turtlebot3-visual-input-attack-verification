# Issue 15 experiment evidence

- [Combined report](REPORT.md) and [machine-readable metrics](metrics.json) cover 60 formal trials, five per controller and condition.
- [Formal LiDAR-only outcomes](formal/runs.json) contain 20 new trials. [Pilot outcomes](pilot/runs.json) contain four separate trial runs excluded from the comparison.
- [Compatibility audit](compatibility.json) records matched protocol settings, source checks and manipulated frame counts before the stop command for every LiDAR-only attack trial.
- [Issue 13 formal records](../issue-13-2026-11-10/formal/runs.json) supply the 40 historical Camera-only and Camera–LiDAR trials.
- [Protocol and reproduction](../../docs/issue-15-evaluation.md) describes the scenes, controls and timing definitions.

Both attacks breached the 0.35 m safety margin in 5/5 Camera-only runs. LiDAR-only and Camera–LiDAR each had 0/5 breaches per attack. Fusion issued its stop command 1.168 simulation seconds earlier on average under replay and 1.208 seconds earlier under occlusion. The lowest sampled clearance lower bounds for replay were 0.855 m for LiDAR-only and 2.071 m for fusion. For occlusion they were 0.886 m and 2.092 m. All three controllers succeeded in all ten clean tasks each, with zero premature stops in each controller's five empty routes.

The runs were collected in two batches with matched settings. Each condition has five repeats and exact seeds were not controlled. The clearance results use independent sampled Gazebo poses. Direct contact events were not logged.
