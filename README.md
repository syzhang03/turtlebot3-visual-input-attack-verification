# turtlebot3-visual-input-attack-verification
Visual Input Manipulation Attacks on TurtleBot3 Waffle Pi and Multi-Sensor Verification

## Current status

The first visual-navigation baseline is implemented for Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic.

- **Implemented:** straight-line motion, red-obstacle detection, automatic stopping, and per-run CSV logs.
- **Validated:** all 10 unit tests passed in Wu Haonan's Ubuntu run. Simulation trials demonstrated clear-route endpoint stopping and repeated red-obstacle stopping. Logs and available screenshots are linked below.
- **Next:** camera-input attacks, LiDAR-based verification, and attack/defense evaluation.

### Documentation and evidence

- [Build and run the baseline](docs/baseline.md)
- [Trial results, screenshots, and evidence limitations](results/baseline-2026-10-27/README.md)
- [Pinned TurtleBot3 source revisions](configs/turtlebot3.repos)
- [AI use declaration and human validation](docs/ai-usage.md)

The research proposal below describes the full project, including planned work that is not yet implemented.

## 1. Question and Motivation

### Security Question

To what extent can manipulated camera inputs cause unsafe behavior in a TurtleBot3 Waffle Pi, and can LiDAR-based cross-sensor verification reliably detect such manipulation?

### Motivation

Autonomous robots rely on sensor inputs to perceive their surroundings and make safety-critical decisions. If an attacker can manipulate camera data before it reaches the perception module, the robot may fail to detect obstacles. Prior work has shown that sensor fusion systems can be vulnerable to adversarial manipulation of individual sensor modalities [1], [2]. Therefore, this project investigates whether LiDAR sensor can detect inconsistencies caused by manipulated camera inputs and prevent unsafe robot motion.

## 2. System and Threat Model

### System and Security Goal

We propose a Gazebo-based TurtleBot3 Waffle Pi testbed with a camera, LiDAR, a visual perception module, a motion controller, and a cross-sensor monitor. In a forward-navigation task, camera-based obstacle estimates will inform the controller's move/stop decisions. The monitor will compare these estimates with LiDAR measurements within their overlapping forward field of view. A detected inconsistency will trigger an alert and a protective stop.

The security goal is to prevent manipulated visual inputs from causing unsafe forward motion while preserving normal task completion.

### Attacker Access and Capabilities

We assume that an attacker has compromised the camera-data interface between image capture and visual perception. The attacker can modify selected image frames received by the perception module, but cannot change the underlying simulated scene.

Planned manipulations include:

- Masking image regions
- Substituting previously captured frames

The attacker aims to conceal an obstacle or create a misleading visual interpretation, causing unsafe movement or unnecessary stops. Attack timing, duration, and the extent of image modification will be varied. Demonstrating the initial compromise is outside the project scope.

### Attacker Limits and Trust Boundaries

LiDAR measurements, controller and monitor code, and the command path to the robot remain trusted. The attacker cannot alter these components, directly issue motion commands, or modify the robot's simulated physical state.

Camera images cross an untrusted input boundary before entering perception; consequently, camera-derived estimates are also treated as potentially unreliable. The monitor uses sensor timestamps and a defined camera–LiDAR geometric alignment to compare corresponding observations.

Simulator ground truth is reserved for evaluation and is unavailable to the controller and monitor. Attacks on LiDAR and simultaneous compromise of multiple sensors are outside scope.

## 3. Technical Approach

We will build on the official TurtleBot3 Waffle Pi platform and simulation packages [3], [4], using `Ubuntu 24.04`, `ROS 2 Jazzy`, and `Gazebo Harmonic` [5]. Separate ROS nodes will handle perception, image manipulation, verification, and motion control.

### Baseline Task

The robot will follow a fixed straight route. An OpenCV detector will use HSV colour thresholding [6], image position, and obstacle area to trigger stopping before contact. Thresholds will be calibrated on clean scenes and fixed before evaluation.

On clear routes, the robot will move at a fixed speed to an odometry-defined endpoint. LiDAR will be logged but will not control the unprotected robot.

### Image Manipulation

A camera relay will forward clean frames, mask selected regions, or substitute recorded images. Attack onset, duration, and mask coverage will be configurable.

Only image content will change: messages will retain current timestamps, while the scene, LiDAR, and control code remain unchanged.

### Verification and Mitigation

The monitor will pair visual decisions and LiDAR scans within a timestamp tolerance and compare corresponding forward regions using known sensor geometry. Clean scene thresholds will map observations to comparable clear/blocked states.

Persistent disagreement will trigger an alert and a protective stop request. The trusted controller, the sole velocity command publisher, will prioritise this request until trial reset. Disagreement indicates inconsistent sensing, not proof of an attack; unnecessary stops will be measured as a performance cost.

### Evidence and Contribution

We will test four configurations (`clean/manipulated input × monitor on/off`) and quantify perception-to-action failures. Our contribution includes:

- The image manipulation pipeline
- The cross-sensor verification monitor
- Controlled experimental analysis

## 4. Evaluation Plan

A successful attack causes a collision or reduces obstacle clearance below the minimum safety threshold. Normal task success means following the prescribed path when clear and stopping at a safe distance when approaching an obstacle.

### Baselines

Four groups are compared:

1. Normal input without defense
2. Attack input without defense
3. Normal input with defense
4. Attack input with defense

Scene, initial position, speed, and random settings are matched across groups.

### Test Cases

These include:

- Obstacle-free movement
- Normal obstacle approaches
- Obstacle approaches with manipulated camera input

Attack cases are tested with defense both disabled and enabled.

### Evaluation Metrics

- Attack success rate
- Normal task success rate
- Minimum obstacle clearance
- Detection latency
- Actual braking latency
- False alarm rate under normal conditions

### Mitigation

The detector checks current visual obstacle assessments against recent frames and compares them with LiDAR observations in the shared forward area at similar timestamps. Historical changes, cross-sensor discrepancies, and their persistence indicate suspected anomalies.

Detection reduces reliance on visual information and triggers a protective stop.

### Conditions

Attack initiation distance, duration, and robot speed will vary. Independent debugging tests will examine LiDAR trigger distance and consecutive confirmation counts before fixing defense parameters for formal evaluation.

### Repeated Trials

Each condition involving random noise will be repeated five times, resetting the environment and program state before each run and matching random seeds across groups. Results will report successes out of total trials and the mean and variance of distance and time metrics.

Missed detections, failed defenses, and unsuccessful attacks will be retained.

## 5. Feasibility

### Required Hardware, Software, Data, and Compute

The project will be implemented entirely in simulation with `Ubuntu 24.04`, `ROS 2 Jazzy`, `Gazebo Harmonic`, TurtleBot3 Waffle Pi simulation packages, `Python`, and `OpenCV`.

No external dataset or deep-learning model training is required. A modern multi-core CPU with 16 GB RAM should be sufficient, and a dedicated GPU is optional. All major software is open-source, so the expected additional cost is approximately $0 when existing computers are used.

### Main Feasibility Risk

The main feasibility risk is reliably determining whether disagreement between camera and LiDAR data indicates an attack rather than normal sensor differences. The sensors have different fields of view, timestamps, and sensing characteristics, which may cause false alarms.

To reduce this risk, the system will compare only their overlapping forward region and combine temporal consistency of recent camera frames with camera–LiDAR cross-sensor consistency.

### Fallback

If fine-grained sensor alignment proves too difficult, we will simplify the experiment to a fixed forward route with a single distinctive obstacle, comparing the camera's binary obstacle/clear decision with LiDAR distance measurements in a predefined forward sector. Persistent disagreement between the two sensors will be treated as suspicious.
