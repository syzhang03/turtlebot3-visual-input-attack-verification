# Three-state Camera–LiDAR controller

Enable this strategy with `-p three_state_enabled:=true` on the existing `camera_lidar_monitor` node. Use the normal Camera–LiDAR parameters and the current scene and sensor bring-up. The parameter defaults below are for the Gazebo straight-road verification.

| State in the event log | Behavior |
|---|---|
| `RUNNING` | Drive at the configured nominal speed while the camera, LiDAR and odometry checks pass. |
| `RESTRICTED` | After startup, a camera sample age over 0.25 simulation seconds can trigger fallback. Limit the speed command to 0.35 m/s while the LiDAR and odometry checks continue to pass. |
| Terminal stop reason | Stop if the LiDAR guard detects an obstacle or its safety inputs become unusable, the monitor confirms inconsistency, or the restricted interval expires. Endpoint completion remains a normal terminal stop. |

The LiDAR guard retains its nominal-speed clearance threshold while restricted, including while the actual car is decelerating. The default restricted interval is at most 2 seconds measured against simulation or steady time, whichever reaches the deadline first. After a gap, the controller requires three agreeing camera–LiDAR pairs with fresh sensor inputs to return to nominal speed. Both samples in each recovery pair must come after entry into the restricted interval. A terminal stop stays latched for the trial.

The event log records state transitions and linear commands. The visual configuration log saves the selected policy, restricted speed, camera age, restricted timeout and recovery pair count. IMU and wheel speed can be recorded by the separate four-sensor collector. The controller continues to use odometry and the LiDAR guard for its motion checks.
