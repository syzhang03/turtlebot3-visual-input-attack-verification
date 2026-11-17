# Issue 7 acceptance evidence

Issue: https://github.com/syzhang03/turtlebot3-visual-input-attack-verification/issues/7

The implementation and automated acceptance evidence are ready for independent teammate review. The agent has not performed or claimed that independent review.

| Criterion | Evidence |
|---|---|
| Obstacle-free endpoint | Five reset empty-world 4 m trials, all GOAL_REACHED with final speed zero; mean endpoint overshoot 0.032277 m |
| Obstacle stop and clearance | Fifteen reset obstacle trials; additionally three evaluation-only Gazebo model-pose checks with conservative clearance lower bounds 0.924436 m clean, 0.926256 m replay, 0.940109 m occlusion, all above 0.35 m |
| Ignore objects outside the driving corridor | Geometry regression test excludes side/rear obstacles; road-scene runs include path-external scenery |
| Missing/stale scans stop motion | Real DDS interruption and repeated-stamp tests; Gazebo scan-interruption runs |
| Unusable scans are not clear | Metadata/range/coverage unit tests and real DDS invalid-scan test |
| Delay/occlusion independence | No camera subscription in controller; reset runs 5/5 per visual attack condition, with attack activation checked before motion |
| Preserve existing functionality | Existing 29 visual baseline tests pass; original visual and attack source files unchanged |
| Log configuration/states/stop reasons | Raw config.json and events.csv retained for every trial |
| Document setup/launch/verification | docs/lidar-only.md and new build/run/verifier scripts |

## Independent simulator position measurement

The evaluator subscribes directly to Gazebo `/world/default/pose/info` using the installed Gazebo Transport 13 / Messages 10 Python bindings. It selects the model named `waffle_pi`. These data are never subscribed to by the controller.

The evaluator verifies that every primitive in the installed pinned Waffle Pi collision model fits within a 0.30 m sphere around the model origin. It uses the sphere's horizontal disk as a conservative enclosure. Distance from the simulator robot position to the red box's axis-aligned footprint (center 4,0; half sizes 0.15,0.25 m), minus 0.30 m, is a lower bound on robot–box separation at recorded poses. The static scene geometry remains unchanged.

This is independent of LiDAR and wheel odometry. It is a conservative pose-derived clearance bound, not a contact sensor or exact mesh distance. The three extra checks are not retroactive ground truth for the prior twenty trials. Their raw `ground-truth.csv` files and summaries are in the `ground-truth-clean`, `ground-truth-delay`, and `ground-truth-occlusion-native` result folders. The failed first ROS-bridge attempt is retained in `ground-truth-occlusion`; it produced zero pose samples and failed before motion, and is not counted as a successful trial.

## Review still required

A different teammate should inspect the implementation and reproduce the documented checks before merging. Review parameter calibration, the no-return convention, the chosen robot envelope, and stopping behavior. No broad robustness claim, multi-sensor benefit claim, or contact-sensor collision rate is made. Final workflow completion occurs after review and merge, not merely when these automated checks pass.
