<div align="center">

# Autonomous Edge-Avoiding Robot

**A mobile robot that explores unknown terrain and never drives off an edge —**
**sensor fusion, layered autonomy, spatial memory, and full test coverage,
simulated in ROS 2 Jazzy + Gazebo Sim 8.**

[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Sim%208%20Harmonic-orange)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![Tests](https://img.shields.io/badge/unit%20tests-27%20passing-brightgreen)](src/bot_script/test)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

![Demo](media/demo.gif)

</div>

## Overview

This is a differential-drive robot that autonomously explores a raised
platform, detecting drop-offs before its wheels reach them and
maneuvering back onto safe ground — the same core problem robot vacuums,
warehouse AGVs, and stair-avoiding delivery robots all have to solve.
Rather than a simple "stop when close to an edge" reflex, the robot runs
a layered behavior stack: proactive steering during normal exploration,
a reactive maneuver when an edge is detected head-on, and an escalating
recovery routine when it detects it's genuinely stuck — all backed by
redundant sensing (LIDAR + four IR-style cliff sensors) and a spatial
memory that steers it away from places it's already learned are
dangerous.

## Highlights

- **Multi-sensor edge detection** — a tilted forward LIDAR for early,
  proactive steering, plus four downward-facing IR-style cliff sensors
  (one per chassis corner) for authoritative, close-range confirmation
  that a single forward-looking sensor structurally cannot provide at an
  actual corner.
- **Layered autonomy (EXPLORE → AVOID → RECOVER)** — smooth proportional
  steering away from edges in the periphery, an immediate reverse-and-turn
  reaction to an edge dead ahead, and a randomized large-scale reorientation
  when the robot detects it's trapped, with maneuver duration that
  escalates the more times it re-triggers in a short window.
- **Spatial hazard memory** — every location the robot has had to react
  to is remembered in world coordinates and actively repels future
  navigation, so the robot steers around known trouble spots instead of
  reacting identically every time it drifts back.
- **27 unit tests, zero simulator required** — all decision-making logic
  (scan interpretation, cliff detection, stuck detection, hazard memory)
  is implemented as pure, dependency-free Python and tested in isolation
  with pytest.
- **Custom robot model built and debugged from scratch** — tuned
  URDF/Xacro inertial properties, a wheel layout redesigned mid-project
  after diagnosing a turning-radius bug, and Gazebo Sim 8 LIDAR/IR sensor
  plugins wired through a `ros_gz` bridge.
- **Full RViz debug visualization** — live LIDAR scan, per-corner
  cliff-sensor status, steering-intent arrow, and a growing hazard map,
  all rendered in real time.

## Demo

<div align="center">
<img src="media/gazebo-sim.png" width="48%" alt="Gazebo simulation view" />
<img src="media/rviz-debug.png" width="48%" alt="RViz debug view" />
</div>

<p align="center"><em>Left: the robot navigating the test platform in Gazebo Sim 8. Right: the RViz debug view — live scan, cliff-sensor status, and hazard map.</em></p>

## How It Works

The robot carries one LIDAR mounted at the front, tilted ~20° downward,
so its beams normally bounce off the floor a short, predictable distance
ahead. Near a drop-off, beams either jump well past that expected
distance or get no return within sensor range at all — either signal
means "edge." Four additional IR-style sensors, one under each corner of
the chassis, look straight down at close range: a single forward-facing
sensor cannot resolve an actual platform corner (turning away from one
edge just points it at the other), so these give a direct, ground-truth
"is there floor under this corner" reading with no geometry inference
needed.

The behavior node fuses both sensor streams into one steering decision
and runs a three-state machine:

```
                 edge in periphery (steer away, no stop)
        ┌──────────────────────────────────────┐
        │                                       │
        ▼                                       │
┌───────────────┐   edge dead ahead     ┌───────┴──────┐
│    EXPLORE     │ ─────────────────────▶│    AVOID     │
│ forward drive, │                       │ reverse, then │
│ proportional   │◀───────────────────── │ turn away;    │
│ steering       │   maneuver complete   │ duration      │
└───────┬────────┘                       │ escalates on  │
        │                                │ repeat traps  │
        │ stuck: no net progress         └──────┬────────┘
        │ despite driving                       │
        │                            N avoids in time window,
        │                          or both front corners at once
        ▼                                       │
┌────────────────────────────────────────────────▼───┐
│                      RECOVER                        │
│  one large randomized rotation (breaks geometric     │
│  symmetry of the trap) → extended forward run        │
└──────────────────────────────────────────────────────┘
```

All decision logic — scan interpretation, cliff-sensor fusion, stuck
detection, and hazard memory — lives in small, pure Python
modules with no ROS or simulator dependency, unit-tested independently
(see [Testing](#testing)).

## Engineering Deep Dive

A few of the harder problems this project involved diagnosing and
solving:

**A silent sensor-interpretation bug that let the robot drive off
edges.** When every beam in a LIDAR sector came back with no return at
all (the clearest possible "no floor" signal — a clean drop-off), the
scan-processing code fell back to a default that read as "floor right
here," exactly inverting the intended behavior at the moment it mattered
most. Diagnosed by working backward from an intermittent failure to the
sensor-fusion math, fixed at the source, and locked in with a regression
test.

**A turning-radius bug traced to wheel placement.** The original wheel
layout drove from the rear axle, so in-place turns pivoted around the
back of the chassis — sweeping the front corners through a wide arc that
could carry them past an edge mid-turn even right after backing away
from it. Fixed by centering the drive axle on the chassis, cutting the
turn-sweep radius by roughly a third and matching how differential-drive
robots are conventionally laid out.

**A sensing gap that no amount of tuning could close.** A forward-facing
LIDAR, however wide its field of view, can only ever look where the
robot is facing — at an actual corner, where two edges meet, steering
away from one just points the chassis at the other. Recognizing this as
a sensor-coverage problem rather than a tuning problem led to adding four
dedicated downward-facing cliff sensors and a corner-trap detector that
routes straight to the large-scale recovery maneuver instead of a small
correction that would just re-trigger.

**Reactive control with no memory repeats its mistakes.** A
fixed-duration reverse-and-turn maneuver has no way to know "I was just
here," so it can send a robot right back toward the same edge repeatedly.
Solved with a lightweight spatial hazard map: every AVOID/RECOVER trigger
is recorded by world position and actively repels future steering,
turning a purely reactive controller into one that improves its own
behavior over a run.

## Tech Stack

| Layer | Tools |
|---|---|
| Robot framework | ROS 2 Jazzy (`rclpy`) |
| Simulation | Gazebo Sim 8 (Harmonic), `ros_gz_bridge` |
| Robot description | URDF/Xacro, SDF |
| Behavior logic | Python 3.12, pure-function/class design for testability |
| Testing | pytest (27 unit tests, no ROS/simulator dependency) |
| Visualization | RViz2 (live scan, markers, hazard map) |
| Build | colcon, ament_cmake / ament_python |

## Project Structure

```
edge_avoiding_robot_ws/
└── src/
    ├── bot_description/   # Robot model (URDF/Xacro), LIDAR + cliff sensors, world (SDF), RViz config
    ├── bot_controller/     # ROS <-> Gazebo topic bridge configuration
    ├── bot_script/          # Behavior node + all unit-tested decision logic
    │   └── bot_script/
    │       ├── edge_avoider.py     # ROS node: state machine, sensor fusion, control loop
    │       ├── scan_utils.py       # Pure scan-interpretation logic
    │       ├── stuck_detector.py   # Pure "no net progress" detector
    │       └── hazard_memory.py    # Pure spatial hazard-memory / repulsion logic
    └── bot_bringup/         # Top-level launch: simulation + spawn + bridge + behavior node
```

## Getting Started

### Prerequisites

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8 (Harmonic)
  ```bash
  sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
                    ros-jazzy-xacro ros-jazzy-robot-state-publisher
  ```

### Build

```bash
cd ~/edge_avoiding_robot_ws
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
```

### Run

```bash
ros2 launch bot_bringup simulated_robot.launch.py
```

This starts Gazebo Sim 8 with a raised test platform, spawns the robot,
starts the ROS↔Gazebo bridge, runs the behavior node, and opens RViz2
with a full debug view. The robot drives forward, detects edges via both
the tilted LIDAR and the corner cliff sensors, backs up, turns, and
continues — never driving off, escaping corners in one decisive move,
and increasingly steering around known trouble spots the longer it runs.

Pass `use_rviz:=false` for a headless run:

```bash
ros2 launch bot_bringup simulated_robot.launch.py use_rviz:=false
```

## Testing

All core decision-making — scan interpretation, cliff detection, stuck
detection, and hazard memory — is pure Python with no ROS or simulator
dependency, so it's covered by fast unit tests that run without building
the workspace:

```bash
pip install pytest
pytest src/bot_script/test/ -v
```

## Tuning

Key parameters live in `bot_script/edge_avoider.py` and can be set at
launch time or live via `ros2 param set`:

| Parameter | Meaning |
|---|---|
| `edge_range_threshold` | Range (m) beyond which a LIDAR beam is treated as "no floor" |
| `cliff_range_threshold` | Range (m) beyond which a corner IR sensor reports "no floor" |
| `forward_speed` / `turn_speed` | Drive speeds |
| `reverse_time_base` / `turn_time_base` | How long to back up / turn after detecting an edge |
| `avoid_trap_count` / `avoid_window_sec` | How many AVOID episodes in how many seconds escalates to RECOVER |
| `stuck_min_displacement` / `stuck_window_sec` | Net (x,y) motion below this over this window counts as "stuck" |
| `hazard_memory_radius` / `hazard_repulsion_gain` | How far a remembered hazard's repulsion reaches, and how strongly it steers |
| `danger_slowdown_factor` | Forward-speed multiplier while any edge/hazard is flagged |
| `geofence_half_extent` | Last-resort hard boundary (m); `0` disables it |

## Roadmap

- A second, flat-mounted LIDAR for general obstacle avoidance (a 3D
  mapping LIDAR is already modeled and publishing, currently unused by
  the behavior node).
- Swap the state machine for a behavior tree or `ros2_control`-based
  controller for smoother motion.
- A coverage/telemetry node logging floor-area explored per run, for
  quantitative regression testing of behavior changes.

## License

[MIT](LICENSE)
