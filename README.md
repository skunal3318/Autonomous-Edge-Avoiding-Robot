# Edge Avoiding Robot — ROS 2 Jazzy + Gazebo Sim 8

[![CI](https://img.shields.io/badge/CI-GitHub%20Actions-blue)](.github/workflows/ci.yml)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E)](https://docs.ros.org/en/jazzy/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Sim%208%20Harmonic-orange)](https://gazebosim.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A from-scratch reimplementation of the edge-avoidance concept from
[MechaMind-Labs/Edge_Avoiding_Robot](https://github.com/MechaMind-Labs/Edge_Avoiding_Robot),
rebuilt for **ROS 2 Jazzy** and **Gazebo Sim 8 (Harmonic)** using the modern
`ros_gz` bridge instead of the old `gazebo_ros_pkgs` + Gazebo Classic stack,
with a layered reactive/deliberative behavior stack, unit-tested scan
interpretation, and CI on every push.

## How it works

- A differential-drive robot carries one LIDAR mounted at the front,
  **tilted ~20° downward**, so its beams normally bounce off the floor a
  short, predictable distance ahead.
- When the robot nears a drop-off, some beams no longer hit anything close
  by, so their reported range jumps well past the expected on-floor
  distance — or gets no return within sensor range at all. Either signal
  means "edge."
- `bot_script/edge_avoider.py` runs a three-state behavior
  (EXPLORE → AVOID → RECOVER, see below): it steers away from edges in its
  peripheral vision before they become urgent, reacts immediately to an
  edge dead ahead, and escalates to a randomized recovery maneuver if it
  detects it's trapped oscillating between two nearby edges.
- Four downward-facing IR-style cliff sensors, one at each chassis
  corner, back up the forward LIDAR with direct "is there floor right
  under this corner" readings. A single forward-looking sensor
  structurally cannot resolve an actual corner (two edges meeting):
  turning away from one edge just points it at the other. The corner
  sensors sidestep that entirely — no geometry inference needed.
- Every AVOID/RECOVER trigger is remembered by world position
  (`hazard_memory.HazardMemory`). That memory actively repels future
  steering and biases which way the robot turns — so once it's learned a
  spot is dangerous, it steers clear of it on later passes instead of
  reacting fresh (and identically) every time it happens to drift back.
- A geofence tied to the platform's known extent is a last-resort
  backstop for the case a sensor reading is missing entirely (bridge
  dropout, etc.) — it is not the primary safety mechanism, the cliff
  sensors and LIDAR are.

## Behavior architecture

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
        │ StuckDetector: no net          └──────┬────────┘
        │ (x,y) progress despite               │
        │ driving                    N avoids in window
        ▼                                       │
┌────────────────────────────────────────────────▼───┐
│                      RECOVER                        │
│  one large randomized rotation (breaks geometric     │
│  symmetry of the trap) → extended forward run        │
└──────────────────────────────────────────────────────┘
```

EXPLORE → AVOID also has a direct-to-RECOVER shortcut not pictured above:
a plain glancing edge (one cliff sensor or one side of the scan flagged)
gets the normal AVOID maneuver, but *both* front cliff sensors tripping
at once is the signature of being wedged into an actual corner — that
skips AVOID entirely and goes straight to RECOVER's larger, randomized
reorientation instead of waiting for AVOID's trap counter to notice the
same thing several bounces later.

All scan interpretation (`scan_utils.analyze_scan`), stuck detection
(`stuck_detector.StuckDetector`), and hazard memory
(`hazard_memory.HazardMemory`) are pure, ROS-free functions/classes,
unit-tested independently of rclpy or a running simulator — see
[Testing](#testing).

## Package layout

```
edge_avoiding_robot_ws/
└── src/
    ├── bot_description/   # xacro robot model, LIDAR sensor, world (SDF)
    ├── bot_controller/     # ros_gz_bridge topic mapping + launch
    ├── bot_script/          # edge_avoider.py — the avoidance behavior node
    └── bot_bringup/         # top-level launch: gz sim + spawn + bridge + node
```

## Prerequisites

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8 (Harmonic) — already installed.
- `ros_gz` bridge packages:
  ```bash
  sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
                    ros-jazzy-xacro ros-jazzy-robot-state-publisher
  ```

## Build

```bash
cd ~/edge_avoiding_robot_ws
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
```

## Run

```bash
ros2 launch bot_bringup simulated_robot.launch.py
```

This starts Gazebo Sim 8 with `edge_world.sdf` (a raised platform with a
clear drop-off), spawns the robot, starts the ROS↔Gazebo bridge, runs the
edge-avoidance node, and opens RViz2 with a debug view: `/scan`, TF, the
steering-direction marker, four small **cliff-sensor spheres** on the
robot itself (green = floor, red = no floor, one per corner, updating
live), and the growing **hazard map** — red spheres marking every
location the robot has had to avoid, visible proof the memory is
working. The robot should drive forward, detect edges via both the
tilted LIDAR and the corner cliff sensors, back up, turn, and continue —
never driving off, escaping corners in one decisive move instead of
oscillating, and increasingly steering around known trouble spots rather
than re-triggering on them.

Pass `use_rviz:=false` to skip RViz for a headless/CI run:

```bash
ros2 launch bot_bringup simulated_robot.launch.py use_rviz:=false
```

## Testing

`scan_utils.analyze_scan()` / `scan_utils.is_cliff()`,
`stuck_detector.StuckDetector`, and `hazard_memory.HazardMemory` are pure
Python with no ROS or simulator dependency, so they're covered by fast
unit tests (27 and counting) that run without building the workspace:

```bash
pip install pytest
pytest src/bot_script/test/ -v
```

CI ([.github/workflows/ci.yml](.github/workflows/ci.yml)) runs this suite
on every push/PR, plus a full `colcon build` + `colcon test` inside a
`ros:jazzy-ros-base` container to catch xacro/launch/package.xml
regressions the pure unit tests can't see.

## Tuning

Key parameters live in `bot_script/edge_avoider.py` and can be set at
launch time or via `ros2 param set`:

| Parameter | Meaning |
|---|---|
| `edge_range_threshold` | Range (m) beyond which a beam is treated as "no floor" |
| `forward_speed` / `turn_speed` | Drive speeds |
| `reverse_time_base` / `turn_time_base` | How long to back up / turn after detecting an edge |
| `avoid_trap_count` / `avoid_window_sec` | How many AVOID episodes in how many seconds escalates to RECOVER |
| `stuck_min_displacement` / `stuck_window_sec` | Net (x,y) motion below this over this window counts as "stuck" |
| `hazard_memory_radius` / `hazard_repulsion_gain` | How far a remembered hazard's repulsion reaches, and how strongly it steers |
| `hazard_memory_size` | Max remembered hazard points (oldest evicted first) |
| `danger_slowdown_factor` | Forward-speed multiplier while any edge/hazard is flagged, to shrink the arc a turn sweeps |
| `cliff_range_threshold` | Range (m) beyond which a corner IR sensor reports "no floor" |
| `geofence_half_extent` | Hard odometry boundary (m), last-resort only; `0` disables it |

If the robot reacts too late or too early, adjust `edge_lidar_tilt` and
the LIDAR's `range/max` in `bot_description/urdf/bot.urdf.xacro` — those
set the "expected floor distance" that `edge_range_threshold` is compared
against. The corner cliff sensors' own mount height/range live next to
them in the same file (`ir_cliff_sensor` macro) if the chassis size
changes.

## Design decisions worth knowing about

- **A sector with *zero* in-range returns is treated as an edge, not
  floor.** `bin_ranges()` used to fall back to `0.0` when every beam in a
  sector came back `inf`/`nan` (no floor found within the sensor's max
  range at all) — which reads as "floor right here" to the threshold
  check. That's the exact moment a clean drop-off produces its strongest
  signal, so it was the direct cause of the robot occasionally driving
  off a platform edge. Fixed in `scan_utils.bin_ranges()`, with a
  regression test (`test_no_return_sector_is_treated_as_edge_not_floor`)
  covering it.
- **Drive wheels sit on the robot's centerline, not the rear axle**, with
  one caster front and one aft instead of both up front. A rear-axle
  drive layout pivots turns around the back, so the front corners sweep a
  wide arc (~0.9 m for this chassis) when rotating — enough to carry a
  corner past an edge mid-turn even right after backing up. Centering the
  drive wheels shrinks that sweep to the chassis's own half-diagonal
  (~0.58 m) and matches how most differential-drive robots are actually
  laid out.
- **The LIDAR's horizontal FOV was too narrow to see the chassis's own
  corners.** At ±0.3 rad (~17°) the forward cone covered only what was
  directly ahead of the sensor; a corner 0.3 m off centerline sits at
  ~31° off-axis by the time it's close enough to matter, so it could
  swing over an edge — especially while curving through EXPLORE's
  proportional steering — without a single beam ever pointing at the
  ground it was about to cross. Widened to ±0.6 rad (~34°) in
  `bot.urdf.xacro`.
- **No memory meant every edge encounter was treated as brand new.** A
  fixed-duration reactive maneuver has no way to know "I was just here" —
  so it could send the robot right back toward the same edge on the next
  pass. `hazard_memory.HazardMemory` records the world-frame position of
  every AVOID/RECOVER trigger; that memory both biases EXPLORE's steering
  away from nearby remembered hazards and informs which way AVOID/RECOVER
  turn when the scan itself is ambiguous (dead-center edge).
- **A single forward-looking sensor can't resolve a corner, no matter how
  wide its FOV.** At an actual platform corner, two edges meet — steering
  away from the one you can see just points the chassis at the other, so
  the robot oscillated in place instead of escaping. This isn't fixable
  by tuning a forward sensor; it needed a different sensor. Four
  downward-facing IR-style cliff sensors, one per chassis corner
  (`ir_cliff_sensor` macro in `bot.urdf.xacro`), give a direct,
  ground-truth "is there floor under this corner" reading — no geometry
  inference, no FOV to be too narrow. This is also how the original
  hardware project this reimplements, and most real edge-avoiding robots,
  actually do it. `_apply_cliff_sensors()` in `edge_avoider.py` fuses
  them into the same `SteeringDecision` the rest of the state machine
  already consumes, and both front sensors tripping together is treated
  as "cornered" — a direct jump to RECOVER's large randomized turn
  instead of a small AVOID nudge that would just re-trigger.
- **The geofence went from primary to last-resort.** It was originally
  the main defense against driving off the platform; now that real cliff
  sensors do that job with actual sensing, a hardcoded coordinate
  boundary only earns its place as a backstop for the case a sensor
  reading is missing entirely (e.g. bridge dropout) — folded into the
  same `SteeringDecision` the state machine already reacts to, not a
  special case.
- **The AVOID maneuver's REVERSE step was blind.** The LIDAR is
  front-mounted and tilted forward, so it sees nothing behind the robot
  while backing up. The rear cliff sensors now cover that blind spot
  directly (with the geofence still checked too, as a fallback); either
  tripping cuts REVERSE short immediately instead of running out the
  fixed duration further into trouble.
- All these fixes are defense in depth, each closing a different gap: the
  scan fix stops false "safe" readings at the sensor-interpretation
  layer, the wheel layout fix shrinks the physical margin an AVOID
  maneuver needs, the widened FOV and corner cliff sensors let the robot
  actually sense its own corners instead of inferring their safety from
  geometry, hazard memory steers around what's already been learned, and
  the geofence is a sensor-independent backstop for what all of the above
  might still miss.

## What differs from the original Humble/Gazebo Classic project

| Original | This version |
|---|---|
| Gazebo Classic | Gazebo Sim 8 (Harmonic) |
| `gazebo_ros_pkgs` plugins | `gz-sim-*-system` plugins + `ros_gz_bridge` |
| `gazebo` command | `gz sim` command |
| Automatic ROS↔Gazebo topics | Explicit bridge config (`bot_controller/config/bridge.yaml`) |

## Next steps you can add

- A second, flat-mounted LIDAR for general obstacle avoidance (the 3D
  mapping LIDAR is already modeled and publishing on `scan_3d`, unused by
  the avoider today).
- Swap the EXPLORE/AVOID/RECOVER state machine in `edge_avoider.py` for a
  behavior tree or `ros2_control`-based controller if you want smoother
  motion.
- A coverage/telemetry node logging floor-area covered per run, useful
  for regression-testing behavior changes quantitatively instead of by eye.

## License

[MIT](LICENSE)
