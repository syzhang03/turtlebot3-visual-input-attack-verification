# Empty-road scene

The `full` scene is the existing `worlds/intersection-white-line.sdf`: houses,
road, white stop line and the red box at `X=4.0 m, Y=0`. The `empty` scene is
`worlds/empty-road-white-line.sdf`: the same houses, road and stop line, with
the red-box visual and collision removed. No other geometry or pose changed.

## Files and launch

| Item | Value |
|---|---|
| Full world | `worlds/intersection-white-line.sdf` |
| Empty world | `worlds/empty-road-white-line.sdf` |
| Launch | `scripts/start_scene.launch.py`, `scene:=full` or `scene:=empty` |
| Helper | `bash scripts/start_scene.sh [full or empty]` |
| Empty visual baseline | `configs/baseline_empty_road.yaml` |
| Empty LiDAR baseline | `configs/lidar_empty_road.yaml` |
| Empty visual runner | `bash scripts/run_empty_road_visual.sh drive` |
| Empty LiDAR runner | `bash scripts/run_empty_road_lidar.sh drive` |
| Passive scene check | `python3 scripts/check_empty_road.py` |

The launch file validates the scene before Gazebo starts. `full` requires the
red visual and collision at `[4, 0, 0.25, 0, 0, 0]`; `empty` rejects any
`red_box_visual` or `red_box_collision`. Both require `road_x_visual` and
`stop_line_visual`. The white stop line remains at `X=3.15 m`; the empty-road
profiles use the odometry endpoint at 4.0 m because there is no red obstacle to
trigger a visual or LiDAR stop.

## Run

Terminal A:

```bash
ROS_DOMAIN_ID=33 bash scripts/start_scene.sh empty
# wait for: SCENE_READY: empty scenery loaded (no obstacle ahead)
```

Terminal B, after `SCENE_READY`:

```bash
# Visual baseline, endpoint stop at 4.0 m
ROS_DOMAIN_ID=33 bash scripts/run_empty_road_visual.sh drive

# or LiDAR-only baseline, endpoint stop at 4.0 m
ROS_DOMAIN_ID=33 bash scripts/run_empty_road_lidar.sh drive
```

Terminal C, while the robot is still at the start pose (before driving), for a
passive check that the forward corridor is clear nearby:

```bash
ROS_DOMAIN_ID=33 python3 scripts/check_empty_road.py --seconds 5
# expected: EMPTY_ROAD_PASS
```

At the start pose the red box in the full scene is at 4.0 m, beyond the 3.5 m
LiDAR range, so this check does not by itself distinguish full from empty. Use
the baseline outcome and the static world tests for that. It is a development
check for nearby obstacles, not a collision proof.

## Acceptance

- `SCENE_READY: empty scenery loaded (no obstacle ahead)`.
- No red box in Gazebo; the road, houses and white line are unchanged.
- `check_empty_road.py` reports `EMPTY_ROAD_PASS` with no finite return nearer
  than 2.0 m in the forward ±10° sector (the robot is stationary at the start).
- Visual and LiDAR baselines reach `GOAL_REACHED` near 4.0 m without a stop
  caused by an obstacle. Keep the default 1.0 m/s and the same initial pose.
  On the full scene, the same visual baseline stops near 2.95 m because of the
  red box; that difference is the behavioural check.
- Preserve the run logs and the scene screenshot; do not compare these runs
  with obstacle-attack trials as if they were matched conditions.

## Host tests

The world difference and launch/config wiring are covered by host tests:

```bash
python3 -m unittest discover -s tests -v
```

Gazebo launch and the passive scene check still require the Ubuntu VM; the
host tests do not prove rendering, collision geometry or `/scan`.

## Limits

- The empty road still contains houses and walls outside the driving corridor.
- The white line is a visual reference; neither baseline detects it.
- The check only looks at the LiDAR forward sector; it does not prove the
  camera view is clear, and it does not replace a simulator collision check.
- The 4.0 m endpoint is an odometry estimate, not a surveyed distance.
