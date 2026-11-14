# Compare the camera stream with the Gazebo viewer disabled

The 2026-11-14 VMware stationary observation showed the same 0.627 s simulation-header gap on both raw and relayed image subscriptions. Previously measured relay preparation/publish/log work was sub-millisecond during a 1.935 s callback interval. These observations narrow the investigation but do not distinguish Gazebo image generation, bridge/DDS delivery and process scheduling.

`start_scene.sh server` is a diagnostic intervention: it omits the separate Gazebo viewer process. The ordinary command still defaults to `gui`. It preserves the installed empty world, installed TurtleBot3 model/spawn/bridges, robot description, initial pose, experiment scene, camera resolution and rate, and existing software-rendering environment. The simulation server still renders sensor images. Do not add `--headless-rendering` or change the rendering backend during this comparison.

The implementation follows the separate server/client actions in the [official Jazzy empty-world launch](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/jazzy/turtlebot3_gazebo/launch/empty_world.launch.py), and reuses the installed [robot spawn and image bridge launch](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/jazzy/turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py). Installed files may differ from upstream; live launch and timing must be checked in the VM.

Stop the old controller, relays and scene before starting a fresh instance:

```bash
ROS_DOMAIN_ID=33 bash ~/camera-lidar-attack/scripts/start_scene.sh server
```

No viewer window is expected. Wait for SCENE_READY. In a second terminal:

```bash
ROS_DOMAIN_ID=33 JOINT_RESULTS_DIR="$HOME/camera-stream-diagnostics" \
bash ~/camera-lidar-attack/scripts/run_joint_attack.sh camera 1.0
```

After REPLAY, observe both image topics for 60 seconds in a third terminal:

```bash
ROS_DOMAIN_ID=33 bash /mnt/hgfs/VM_share/camera-lidar-attack/scripts/check_camera_streams.sh --seconds 60
```

Keep the motion controller off for this diagnostic. Compare header gaps and wall gaps with the GUI observation. A single quiet observation is not proof of a repair; the checker adds subscription load, and the controller adds more work when driving. If the improvement persists, repeat the clean driving condition before the attack conditions, use the same launch mode for every matched trial, and retain the failed trials as SENSOR_TIMEOUT outcomes. Do not loosen the controller's freshness thresholds on just one condition.

For a causal GUI-on versus GUI-off comparison, reset both observations to the same initial robot pose and use the same other running processes. The previous observation may have been taken after a drive, so improvement after a fresh server-only launch alone does not establish the viewer as the root cause.

This option is intended to reduce viewer work, not guarantee a real-time factor or remove all image stalls. It does not provide collision evidence or screenshots. GUI startup remains available with `start_scene.sh` after stopping the previous scene; that command resets the scene rather than attaching to an existing run.

Host validation is Python compilation and Bash syntax only. ROS launch, sensors, and timing in VMware have not been verified by the Windows host.

## Native Gazebo / ROS comparison

The server-mode observation later still contained a 0.561 s relayed header gap and a shared 0.528 s raw/relayed gap. It did not establish stable input under the 0.5 s freshness requirement. More received frames alone are not proof of a repair or a causal viewer effect.

Keep the current server scene and relay running, with the motion controller off:

```bash
ROS_DOMAIN_ID=33 bash /mnt/hgfs/VM_share/camera-lidar-attack/scripts/check_camera_streams.sh --seconds 60 --gazebo
```

The optional native subscriber uses the [Gazebo Transport 13 Python subscribe API](https://github.com/gazebosim/gz-transport/blob/gz-transport13/python/test/pubSub_TEST.py) and [Gazebo Image header](https://github.com/gazebosim/gz-msgs/blob/gz-msgs10/proto/gz/msgs/image.proto). It requires the installed `gz.transport13` and `gz.msgs10` bindings, and the same GZ_PARTITION as the server. Missing bindings cause an explicit error before observation, rather than silently substituting a two-stream test. The native topic defaults to `/camera/image_raw`; it can be changed with `--gazebo-topic` if the model uses another topic.

It retains receive times and message stamps only, plus first-message dimensions/format, CPU count/load and memory snapshots, ROS publisher QoS and the diagnostic process's RMW implementation. Pixel data is not saved. Like any independent subscriber, it adds load and may itself miss or queue messages. No motion commands, settings or model edits are made.

If native intermediate timestamps are observed within a ROS gap, prioritize investigation of the bridge, ROS transport and subscription scheduling. If a matched gap is also observed natively, investigate the simulator, native transport and common scheduling. Unmatched boundaries, time resets or absent native samples cannot localize the failure. Native transport observations are not direct instrumentation of the camera's frame-generation function.

Only helper calculations and Python compilation have been checked on the Windows host; live native subscription and matching require the VM run.

## Reliable camera input candidate

The native comparison in `20261114T112922.307768Z` observed 562 native images with a maximum 0.034 s header gap, versus 109 raw ROS and 116 relayed images. During a common 0.990 s ROS gap, 29 intermediate native timestamps were received. This localizes that observed interruption after native delivery, without proving a particular bridge/DDS/scheduling cause. The raw ROS publisher advertised RELIABLE, whereas the relay input and diagnostic raw subscription were BEST_EFFORT. These policies are compatible, but do not establish a reliable subscriber connection; see the [ROS QoS compatibility rules](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst). Discovery's UNKNOWN history/depth 0 is not evidence of a literal zero-size publisher queue.

The candidate update adds `input_reliability` to the camera relay. Its standalone default remains `best_effort` for compatibility; the joint launch explicitly chooses `reliable`, KEEP_LAST depth 1, in every condition. Output reliability stays reliable with depth 1. A bounded DDS receive history avoids deliberately adding an unbounded backlog, but reliable transport may still stall or queue elsewhere; live timing must pass the unchanged freshness requirements. Replay content/header selection is unchanged.

The relay config and joint manifest record the selected transport, and the trial summary checks that they match. Keep previous runs and do not pool different transport versions as matched experimental replicates. After this candidate passes stationary observation, repeat clean driving validation before the two attack groups. The physical scene, image dimensions, calibrated D=0.788 profile and controller watchdogs are not changed by this update.

Stop the old relay first, then install and rebuild from the shared source:

```bash
bash /mnt/hgfs/VM_share/camera-lidar-attack/scripts/install_camera_transport_update.sh
ROS_DOMAIN_ID=33 JOINT_RESULTS_DIR="$HOME/camera-stream-diagnostics" \
bash ~/camera-lidar-attack/scripts/run_joint_attack.sh camera 1.0
```

Wait for `Input reliability=reliable; queue depth=1` and REPLAY. In another terminal:

```bash
ROS_DOMAIN_ID=33 bash /mnt/hgfs/VM_share/camera-lidar-attack/scripts/check_camera_streams.sh \
  --seconds 60 --gazebo --raw-reliability reliable
```

The installer runs replay selection, ROS message-boundary and real-DDS tests. The latter exchange full 640×480 RGB images through the node and verify zero-delay identity, historical payload/current header behavior and the actual requested input QoS. These tests do not prove sustained loaded Gazebo performance. On the Windows host only the 11 replay-core tests run; the two ROS message and three DDS tests require the VM.
