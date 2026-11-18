# TurtleBot3 three-state reliability evaluation

200/200 scored trials. 0 previous invalid attempts for the completed trial IDs were retained.

The five inputs are nominal, timestamp-preserving camera transport delay, a physical camera visor rendered by Gazebo, temporary LiDAR unavailability, and simultaneous camera and LiDAR unavailability.
The delay and unavailability windows begin 1.7 simulation seconds after motion and last 0.5 seconds. The visor is attached throughout its trial. The 0.5 second noise window starts at 1.0 second to separate noise from the temporary reliability condition.
Both scenes use the same three-state controller. The nominal and restricted speed commands are 1.0 m/s and 0.35 m/s. The controller uses its recorded safety inputs, and clearance evaluation uses independent simulator geometry.

| Scene | Input | Noise | Trials | Alerts | Restricted | False stops | Task success | Min. clearance lower bound (m) |
|---|---|---|---:|---:|---:|---:|---:|---:|
| empty | clean | off | 10 | 1 | 6 | 0/10 | 10 | N/A |
| empty | clean | on | 10 | 0 | 9 | 0/10 | 10 | N/A |
| empty | camera-latency | off | 10 | 0 | 10 | 0/10 | 10 | N/A |
| empty | camera-latency | on | 10 | 0 | 10 | 0/10 | 10 | N/A |
| empty | camera-visor | off | 10 | 0 | 6 | 0/10 | 10 | N/A |
| empty | camera-visor | on | 10 | 0 | 9 | 0/10 | 9 | N/A |
| empty | lidar-unavailable | off | 10 | 0 | 2 | 0/10 | 0 | N/A |
| empty | lidar-unavailable | on | 10 | 0 | 4 | 0/10 | 0 | N/A |
| empty | both-unavailable | off | 10 | 0 | 10 | 2/10 | 0 | N/A |
| empty | both-unavailable | on | 10 | 0 | 10 | 1/10 | 0 | N/A |
| obstacle | clean | off | 10 | 0 | 6 | 0/10 | 10 | 0.747 |
| obstacle | clean | on | 10 | 0 | 4 | 1/10 | 8 | 0.848 |
| obstacle | camera-latency | off | 10 | 0 | 10 | 0/10 | 10 | 0.779 |
| obstacle | camera-latency | on | 10 | 0 | 10 | 0/10 | 10 | 0.857 |
| obstacle | camera-visor | off | 10 | 9 | 6 | 9/10 | 0 | 2.096 |
| obstacle | camera-visor | on | 10 | 9 | 3 | 10/10 | 0 | 2.098 |
| obstacle | lidar-unavailable | off | 10 | 0 | 5 | 0/10 | 0 | 1.858 |
| obstacle | lidar-unavailable | on | 10 | 0 | 5 | 1/10 | 0 | 1.863 |
| obstacle | both-unavailable | off | 10 | 0 | 10 | 1/10 | 0 | 2.036 |
| obstacle | both-unavailable | on | 10 | 0 | 10 | 3/10 | 0 | 2.031 |

False-stop denominators include outcomes with a determinate stop judgment. A confirmed stop under unavailable safety observations is a protective stop. An aggregate consistency alert has no validated per-sensor attribution.
The grouped task-success column uses route completion on the empty road and the expected normal obstacle stop plus verified clearance in the obstacle scene. The clearance columns independently assess safety for protective and other stops.
The separate sensor breakdown includes confirmed delivered noise and reliability exposure. Trials ending before exposure remain in the scheduled-condition results.

- [Grouped metrics](group-metrics.csv), [noise-channel breakdown](noise-channel-metrics.csv), and [per-trial results with raw evidence directories](formal/trials.csv).
- [Frozen plan and settings](formal/protocol.json). Per-trial native model poses, synchronized four-sensor data, input arrival and acceptance logs, state transitions and velocity commands are in the evidence archive linked below.
- Contact events were not instrumented. The clearance values are geometric bounds observed during the evaluated intervals.

- [camera-latency camera example](camera-latency-example.gif). Trial obstacle-camera-latency-04-on.
- [camera-visor camera example](camera-visor-example.gif). Trial obstacle-camera-visor-04-on.
- [both-unavailable camera example](both-unavailable-example.gif). Trial obstacle-both-unavailable-04-on.

## Completion and interpretation

All 200 planned trials are valid, with 10 in each of the 20 scene, input and noise groups. The audited collection hashes match the frozen protocol. No formal attempt required a retry. Reliability condition evidence was present before the stop in every trial. The control input has no scheduled reliability interruption. Of the 100 scheduled noise trials, 99 had confirmed changed inputs before the stop. The unexposed trial was obstacle-clean-04-on. It is retained in its assigned group with a zero exposure flag.

Every one of the 100 obstacle trials had a conservatively bounded minimum clearance above 0.35 m. The lowest bound across the entire approach was 0.747 m, and the lowest at settling was 0.748 m. These are bounds from sampled independent Gazebo poses. In the camera transport-delay condition, all 40 runs entered restricted mode, all 20 empty-road runs reached the goal and all 20 obstacle runs ended in OBSTACLE_STOP. Camera physical occlusion produced 19 adjudicated false stops in 40 trials, all in the obstacle scene under the recorded fresh-input and conservative clearance criteria.

The nominal input groups had 25 restricted runs in 40, including 12 of the 20 nominal trials without added noise. These are measured events with the camera freshness policy enabled. They mean restricted mode cannot be used as a proxy for injected fault detection. Up to three independent Gazebo instances ran concurrently. Timing variation under this collection load was not isolated experimentally. This cohort evaluates the new policy across conditions and has no matched old-policy comparator.

## Review evidence and timing audit

- [Trial evidence archive](formal/trial-evidence.zip) contains selected logs and scores for all 200 runs. [Archive checksum and omissions](formal/evidence-archive.json) identifies the preserved data. Full per-trial camera frames, raw noise arrays and original Gazebo launch logs remain in the local experiment workspace.
- The completed collection used controller source `aef5380` and the local reliability harness. The controller was integrated on top of `6690563` afterwards. The 200 trials are the frozen experimental record. The integrated controller has separate test results.
- Recorded consistency alerts occurred in 19 trials. Seventeen occurred at or before the first stop command, all in the obstacle approach with the physical visor. The alert in one clean trial occurred after endpoint stopping.
- Eight of the 28 formal false stops occurred during LiDAR-only or dual-sensor unavailability at a recorded scan age within 1 ms of the 0.5 s limit. The original adjudication is retained. These eight results are sensitive to timing and should be interpreted separately from the 19 obstacle-visor false stops.
- Of 100 noise pairs, seven had a false stop only in the noise trial and three had a false stop only in the trial without added noise. One scheduled noise run ended before confirmed exposure. This is a descriptive paired comparison across the tested conditions.

## Integrated branch validation

Rebuilt the visual baseline, LiDAR controller, and Camera–LiDAR monitor on ROS Jazzy. The fusion and LiDAR test suites passed with the opt-in mode and the existing image delivery profiles.

| Gazebo smoke condition | State sequence after launch | Independent result |
|---|---|---|
| empty-camera-latency | WAITING → RUNNING → RESTRICTED → RUNNING → GOAL_REACHED | Completed the 4 m route |
| obstacle-both-unavailable | WAITING → RUNNING → RESTRICTED → SENSOR_TIMEOUT | Protected stop with a 2.061 m minimum clearance lower bound |
