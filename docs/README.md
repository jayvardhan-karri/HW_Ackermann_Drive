# LIMO MoCap Control Suite

A hardware-agnostic, off-board trajectory tracking suite for the AgileX LIMO. This package bridges PhaseSpace Motion Capture (MoCap) data to standard ROS 2 Odometry, allowing you to draw trajectories interactively and track them using highly-tuned control algorithms (PID, LQR, LTV-MPC, and NMPC) without modifying the robot's onboard software.

> [!IMPORTANT]
> **Robot heading = the vector from marker 1 to marker 0.**
> Marker **0** (`front_id`) must be on the **front** of the LIMO and marker **1** (`rear_id`) on the **rear**, on the robot's centre line. The bridge computes `yaw = atan2(y0 - y1, x0 - x1)`, and the position is the midpoint of the two markers. The GUIs draw the robot with this convention: a black arrow from the **blue dot (M1, rear)** to the **red dot (M0, front)**. If the markers are mounted the other way round, swap them with `-p front_id:=1 -p rear_id:=0` instead of re-mounting.

## Project Architecture & File Breakdown

The package is modular, separating trajectory generation, state estimation, safety, and control math:

*   **`phasespace_bridge.py`**: The Hardware Abstraction Layer. Subscribes to `/phasespace/markers` (`phasespace_msgs/Markers`), finds markers by their `id`, computes the LIMO's centroid and yaw (marker 1 → marker 0), and publishes `/hardware_odom`. It also builds the obstacle map, from a flat list and/or live from rim markers, and publishes it on `/hardware_obstacles`.
*   **`interactive_builder.py`**: A Matplotlib-based GUI. Click and drag to draw a path, apply B-spline smoothing, and publish it to `/reference_path`. It shows the live robot pose and heading and the obstacles while you draw.
*   **`live_plotter_node.py`**: A Matplotlib-based live view of a run: reference path, robot trail, robot glyph with heading, and obstacles.
*   **`limo_test_suite.py`**: Open-loop actuator test with a GUI. It plays a test path (straight, left circle, figure-8, or a drawn path) as `/cmd_vel` commands and shows, from behind the robot, where the LIMO should go and how its front wheels should be steered. No MoCap or feedback needed; the robot can sit with its wheels in the air.
*   **`ros_compat.py`**: Foxy / Humble shim (`init_ros`), so the same code runs on both.
*   **`plot_helpers.py`**: Shared drawing code for the GUIs: lab grid with red axes for the projector, robot glyph, obstacles, controller frame, Ctrl-C handling.
*   **`trajectory_subscriber.py`**: A mathematical utility class used by all controllers. It subscribes to `/reference_path` and calculates local derivatives (curvature $\kappa$ and reference velocity $v_{\text{ref}}$). It also detects whether the path is a closed loop or an open path.
*   **`obstacle_cbf.py`**: The Control Barrier Function (CBF) safety filter, used by all four controllers. Using the known obstacle positions (no LiDAR), it adjusts speed and steering by the smallest amount that keeps the robot clear, going around obstacles on the path.
*   **`run_manager.py`**: Everything a controller does besides steering, shared by all four: reach the path start, count laps, watch the errors, obey KILL, log each run, wait for the next path.
*   **`approach_planner.py`**: Dubins paths: the shortest way an Ackermann robot can get onto a path.
*   **`stats_lap.py`**: Per-lap numbers: error, time at the steering limit, steering rate.
*   **`mpc_obstacles.py`**: Obstacles as constraints inside LTV-MPC and NMPC (`obstacle_mode:=constraint`), instead of the CBF filter.
*   **`track_tools.py`**, **`track_publisher.py`**: The capsule track, and the node that sends it instead of a drawn path.
*   **`run_laps.py`**, **`report_lap.py`**: The lap test sequence and its report. **`sim_limo.py`**: stand-in robot for rehearsing without the lab.
*   **Controllers** (subscribe to `/hardware_odom` and `/hardware_obstacles`):
    *   **`pid_tracker_node.py`**: Kinematic Ackermann PID with Curvature Feedforward.
    *   **`lqr_tracker_node.py`**: Infinite-Horizon Continuous Algebraic Riccati Equation (ARE) tracker.
    *   **`mpc_tracker_node.py`**: Linear Time-Varying (LTV) MPC, solved by backward Riccati recursion.
    *   **`nmpc_tracker_node.py`**: Full Nonlinear MPC using SLSQP optimization.

## Prerequisites

*   **OS / ROS 2:** Ubuntu 22.04 + Humble, or Ubuntu 20.04 + Foxy
*   **Python Dependencies:** `numpy`, `scipy`, `matplotlib`
*   **`phasespace_msgs`:** the lab's PhaseSpace message package must be built and sourced (the bridge imports `phasespace_msgs.msg.Markers`).

## ROS 2 Foxy / Python 3.8

The package runs unchanged on **Foxy (Ubuntu 20.04, Python 3.8)** and **Humble (Ubuntu 22.04, Python 3.10)**. On a new machine:
```bash
source /opt/ros/foxy/setup.bash      # or humble
python3 tools/check_deps.py          # what is installed / missing (installs nothing)
./tools/setup_ros2.sh                # apt-installs what is missing (asks first), builds, re-checks
```
Do **not** install Python 3.10 for Foxy: Foxy's `rclpy` only works with Python 3.8. See [COMPATIBILITY.md](COMPATIBILITY.md) for the full dependency list, what was changed, and how it was tested.

## Obstacle avoidance (CBF)

There are **no obstacles by default**. Give them on launch as a flat list, by rim markers, or both:
```bash
ros2 launch limo_mocap_suite hw_tracking.launch.py controller:=nmpc obstacles:="[1.97, 0.81, 0.35]"
ros2 launch limo_mocap_suite hw_tracking.launch.py controller:=nmpc obstacle_markers:="[(2,3,4,5),(6,7,8,9)]"
```
*   **Rim markers:** 4 markers evenly spaced around the top edge of each cylinder. Centre = mean of the 4, radius = mean distance from the centre, updated live if the obstacle moves. If a marker is occluded, the last known circle is kept. A group must be fully tracked once before it exists, and the bridge warns until then, so **check that the obstacle appears on the live plotter before driving**.
*   **Behaviour:** the robot keeps its tracked point `robot_radius + safety_margin` (0.30 m) from each cylinder's edge, goes around an obstacle on its path, and rejoins. It stops (`CBF_STOP`) only if there is no safe way forward. Telemetry logs `min_clearance` and `safety_status`.
*   **Space needed:** the LIMO cannot turn on the spot, so going around needs room. A gap narrower than about 0.6 m between obstacles, or an obstacle against a wall, can make it stop.
*   **Why a filter by default:** the CBF adds the same safety constraint to all four controllers, after each one has computed its command. The LTV-MPC and NMPC can instead take the obstacles as constraints inside the solver (`obstacle_mode:=constraint`, `mpc_obstacles.py`): they then plan round the obstacle over their look-ahead and turn earlier and more smoothly, where the CBF clamps the heading at the last moment and the robot drives a straight tangent to the safety circle. `obstacle_lap.launch.py compare_modes:=true` runs both MPCs both ways for comparison. PID and LQR have no look-ahead, so they always use the CBF.

## `phasespace_msgs`

The bridge imports `phasespace_msgs`, so **the machine running `phasespace_bridge` must have it built and sourced**. ROS 2 does not learn message definitions over the network. So:
*   Same machine as `phasespace_bringup`: source that workspace, and no copy is needed.
*   Different machine: build just the `phasespace_msgs` folder there, from the **`mocap-ros2` branch** of [phasespace-mocap-ros](https://github.com/FocasLab/phasespace-mocap-ros/tree/mocap-ros2) (`main` is ROS 1).

`python3 tools/check_deps.py` tells you whether it is found. Marker fields are in metres, already converted by the bringup node.

> [!WARNING]
> **Check the direction of yaw once in the lab.** The bringup node maps markers as `x = m.x`, `y = m.z`, `z = m.y` with no sign change, unlike its rigid bodies, which use `y = -z`. Depending on the lab's calibration, marker coordinates could be mirrored. Rotate the robot counter-clockwise by hand (seen from above) and watch `ros2 topic echo /hardware_odom`: **the yaw must increase**. If it decreases, every controller would steer the wrong way. Do not drive until this is resolved.

## Build Instructions

Ensure you are in the root of your hardware workspace (e.g., `HW_Ackermann_Drive`):

```bash
colcon build --symlink-install --packages-select limo_mocap_suite
source install/setup.bash
```

## Usage

Because the control stack runs off-board, the base station and the LIMO must share the same `ROS_DOMAIN_ID`.

> [!IMPORTANT]
> **The LIMO's topics are namespaced.** On the lab robot, `ros2 topic list` shows `/limo1/cmd_vel`, `/limo1/imu`, and so on. The nodes publish to `/cmd_vel`, so they must be remapped, otherwise the robot never receives a command and simply does not move. Both launch files do this through `robot_ns` (default **`limo1`**):
> *   `robot_ns:=limo2` for another robot
> *   `robot_ns:=none` for a robot without a namespace (plain `/cmd_vel`)
>
> With `ros2 run`, add the remap yourself: `--ros-args -r /cmd_vel:=/limo1/cmd_vel` (and `-r /imu:=/limo1/imu` for PID/LQR/MPC; it is unused, so optional). In MATLAB set `P.ros.cmd_topic`. Only `/cmd_vel` and `/imu` belong to the robot; `/hardware_odom`, `/hardware_obstacles` and `/reference_path` come from this package and are never remapped.
>
> Check: `ros2 node info /nmpc_tracker_node` must list `/limo1/cmd_vel` under *Publishers*.

### First: the lab check

Run this before any tracking, each time the markers, the robot or the mocap calibration change. It finds what can't be seen from a desk. It reports and suggests; it changes nothing.

With the robot still (it doesn't move):
```bash
ros2 launch limo_mocap_suite hw_check.launch.py
```
Then the slow test drive, at 0.15 m/s for about 30 s. Keep 3 m clear around the robot; it stops by itself if it gets further than that from where it started (`max_range`):
```bash
ros2 launch limo_mocap_suite hw_check.launch.py drive:=true ref_offset:=<your value>
```

| It checks | If wrong |
|---|---|
| Mocap rate, gaps, position and heading jitter | WARN |
| Anything subscribed to the command topic | FAIL: robot off, wrong `ROS_DOMAIN_ID` or `robot_ns`. It won't drive |
| Forward command makes the robot move | FAIL, and stops |
| Yaw points along the driving direction | FAIL, and stops: about 180° means `front_id` and `rear_id` are swapped |
| A left command turns left | FAIL, and stops: the mocap frame is mirrored, and tracking would run away |
| Speed, steering trim, curvature left and right, against the model | WARN |
| `ref_offset`, measured from the driving | WARN, with the measured value. It's rough, so confirm with a tape measure |
| Tightest real radius each way | WARN, with the real steering limit |
| Whether the robot stops by itself when commands stop | WARN: if it doesn't, a lost WiFi link leaves it driving, so keep the hardware stop in reach |

The result prints at the end and goes to `evaluation/hw_check_<date>_<time>/report.json`. KILL in the live view and Ctrl-C both stop it. If a bridge is already running, add `bridge:=false`.

To rehearse with no robot or mocap, using simulated faults: `python3 tools/hw_test/run_hw_tests.py`.

### What happens when you press Publish

1. **Getting there.** If the robot is more than 0.15 m from the path start, or more than 30° off its heading, it drives there first along a Dubins path: arcs of `approach_radius` (0.60 m) joined by a straight, at `approach_speed`. An Ackermann robot cannot turn on the spot, so it may have to loop around: starting just past the point needs a loop of about 4 m. The way there avoids known obstacles and stays inside the lab.
2. **Driving.** An open path runs once. A closed path runs `laps` times (default 1, `laps:=0` until stopped). Laps are counted by forward progress along the path.
3. **Afterwards.** The robot stops and the controller **stays running**: draw and publish another path and it goes again. Only Ctrl-C ends it. A new path during a run replaces the old one.

### Lab day: lab track and the record/compare suites

Step by step in [`LAB_GUIDE.md`](LAB_GUIDE.md). Lab track: radius 1.0 m, straights 4.0 m, centre (3.25, 1.25), inside the mocap area x -0.5..7.0, y -0.5..3.0. The robot drives itself to the race start first. Not tested on the real LIMO yet; `sim:=true` rehearses with the stand-in robot.
```bash
ros2 launch limo_mocap_suite record_lap.launch.py controller:=lqr
```
```bash
ros2 launch limo_mocap_suite compare_lap.launch.py
```
```bash
ros2 launch limo_mocap_suite obstacle_lap.launch.py
```
Logs are copied to the receiving machine with `tools/send_logs.sh`.

### Logs: the `evaluation/` folder

Every run gets its own folder in `evaluation/`, relative to where you launched: `evaluation/<date>_<time>_<controller>_<path_name>/`, with `telemetry.csv` and `summary.json` (result, reason, stats per lap). Nothing is overwritten. Make a report from any folder holding runs:
```bash
ros2 run limo_mocap_suite report_lap evaluation
```

### Capsule track: one controller

Instead of drawing, give `hw_tracking.launch.py` a curve radius. It sends a capsule track (two straights, two half circles) once the controller is ready, and the drawing window stays closed. Without `track_radius`, drawing works as always.
```bash
ros2 launch limo_mocap_suite hw_tracking.launch.py controller:=lqr track_radius:=1.0 laps:=3
```

### Lap test: all controllers on three radii, with a report

```bash
ros2 launch limo_mocap_suite run_laps.launch.py
```

For each radius, each controller (`pid,lqr,mpc,nmpc`) in turn drives to the start and does 3 laps. It warns 5 s before moving. At 0.30 m/s the whole test takes roughly 40 minutes. Shorten it with the arguments below.

| Radius | Why |
|---|---|
| 1.0 m | Well inside the steering limit: every controller should work, so they can be compared |
| 0.5 m | At the limit |
| 0.4 m | Beyond the limit (about 0.46 m): the robot must drift, which shows which controller recovers best |

Results go to `evaluation/run_laps_<date>_<time>/`: `report.md` and `comparison.png` compare the radii, and `R1/`, `R0.5/`, `R0.4/` each have their own report. A run that cannot get round a tight track within its time limit is recorded, and the test goes on.

```bash
ros2 launch limo_mocap_suite run_laps.launch.py track_radii:=1.0 controllers:=lqr,nmpc laps:=2
```
Rehearse with no robot and no lab:
```bash
ros2 launch limo_mocap_suite run_laps.launch.py sim:=true robot_ns:=none
```

### Safety while driving

**Always on:**

| Trigger | What happens |
|---|---|
| **KILL** button in the live view | Every controller stops and refuses to move until **RE-ARM** and a new path. A lap test ends there. The button depends on the window and ROS running: **it is not a hardware stop**. Keep the LIMO's own stop within reach |
| No odometry for 0.5 s | Zero command until odometry returns. Leaving the MoCap area ends up here |
| A marker jumps, or the two markers aren't their usual distance apart | The bridge drops the frame, so a ghost marker can't swing the heading. If frames keep being dropped, it counts as no odometry |
| A controller stops reporting (lap test) | Zero command; the test stops |

**Extensions, off by default:**

`guard:=true` adds stops that can interrupt a run. It is off so the controller alone decides what happens when the robot drifts, which is what the 0.4 m track measures.

| Trigger, with `guard:=true` | What happens |
|---|---|
| Error above `slow_cte` (0.15 m) | Slows down, to 40 % at `rejoin_cte` |
| Error above `rejoin_cte` (0.35 m), or heading more than 75° off | Plans a drivable way back onto the path, ahead of where it left. Waits while the CBF is going around an obstacle |
| Error above `abort_cte` (1.0 m), more than 3 rejoins, left `lab_x`/`lab_y`, no way back, approach too slow, or no odometry for 3 s | Stops. In a lap test, the remaining controllers do not run |

Lap-to-lap learning was removed; [FUTURE_lap_learning.md](FUTURE_lap_learning.md) has its design, results and code. `approach:=false` (`hw_tracking.launch.py` only) skips the drive to the start and tracks straight away.

### Quick start (one terminal)

```bash
source install/setup.bash
ros2 launch limo_mocap_suite hw_tracking.launch.py controller:=nmpc   # /cmd_vel -> /limo1/cmd_vel (robot_ns default)
```

This starts the bridge, the controller, the builder and the live plotter. Draw a path in the builder, click **Smooth Path**, then **Publish**. With the lab obstacle:
```bash
ros2 launch limo_mocap_suite hw_tracking.launch.py controller:=nmpc \
    obstacles:="[1.970681, 0.80659, 0.35]"
```

> [!IMPORTANT]
> **Measure `ref_offset` before tracking runs.** Measure along the robot's centre line from the rear axle to the midpoint of markers 0 and 1, and pass it with `ref_offset:=<metres>`. In a fake-robot test, a 5 cm error left 1.6–5.1 cm RMS, depending on the controller.
>
> **Start the controller before clicking Publish.** The path is sent once; a controller started later never receives it and waits. Clicking **Publish** again fixes it.

### Step by step (separate terminals)

**1. PhaseSpace bridge**
```bash
ros2 run limo_mocap_suite phasespace_bridge --ros-args -p "obstacles:=[1.970681, 0.80659, 0.35]" -p "obstacle_markers:=[2,3,4,5]"
```

**2. A controller.** It idles until a path arrives.
```bash
ros2 run limo_mocap_suite nmpc_tracker --ros-args -r /cmd_vel:=/limo1/cmd_vel
# alternatives: pid_tracker, lqr_tracker, mpc_tracker (same -r remap; add -r /imu:=/limo1/imu if you like)
```

**3. Path builder.**
```bash
ros2 run limo_mocap_suite interactive_builder
```
*   Click and drag on the canvas to draw your route.
*   Click **Smooth Path** to apply B-spline curvature smoothing.
*   Click **Publish**. The idling controller locks on and begins tracking.
*   A stroke that ends where it started is a **closed loop** (driven repeatedly). Any other stroke is an **open path**, and the controller stops by itself at its end.

**4. Live plotter** (optional).
```bash
ros2 run limo_mocap_suite live_plotter
```

> [!NOTE]
> **Use the same `rezero_odom` value everywhere** (controller, builder, live plotter).
> *   `false` (default): everything is in the absolute PhaseSpace frame, the same frame as the obstacle map.
> *   `true`: coordinates are relative to the robot's pose when each node received its first odometry. The canvas origin is then the robot, facing +X. Start all nodes together, before the robot moves, so they agree on that pose.

### LIMO test suite (wheels in the air, no feedback)

Checks that the LIMO's wheels do what `/cmd_vel` says, before any closed-loop run. Lift the robot so its wheels spin freely (or put it on the floor: it just drives the path blind), stand **behind** it, and compare with the screen.

```bash
ros2 launch limo_mocap_suite limo_test.launch.py                       # sends to /limo1/cmd_vel
ros2 launch limo_mocap_suite limo_test.launch.py path:=circle speed:=0.15 robot_ns:=limo1
# or without launch (remap yourself), starting straight away:
ros2 run limo_mocap_suite limo_test_suite --ros-args -p robot_ns:=limo1 -p path:=circle -p speed:=0.15 -p autostart:=true
```
`limo_test.launch.py` arguments: `robot_ns` (default `limo1`, `none` = plain `/cmd_vel`), `path` (default `figure8`), `speed` (default `0.20`), `view` (default `behind`).

*   Pick a path with the radio buttons (**straight**, **circle**, **figure8**, **drawn**), then press **Start**. After a 3 s countdown the commands begin. **STOP** (or closing the window, or Ctrl-C) sends zero velocity immediately.
*   **Left panel, view from behind:** forward (+X) points **up** and left (+Y) points **left**, so the screen matches the robot in front of you. The grey dashed line is the test path and the orange line is the motion the commands produce. The robot glyph's arrow points along the last traced segment; it starts pointing straight up and turns with the path, as an Ackermann robot would.
*   **Right panel, wheels seen from behind:** the front wheels are drawn at the commanded steering angle (turned left = left turn), with a green arrow for forward speed. Below it: countdown or time, speed, steering (`LEFT` / `RIGHT` and degrees), yaw rate, and a red warning if the steering is saturated.
*   What to expect: **straight** has wheels centred for the whole run. **circle** has wheels turned left (14° with the default radius) for one full loop. **figure8** turns left for one loop, then switches to right for one loop. **drawn** replays the last path published on `/reference_path` (e.g. from the builder), starting from the robot heading forward.

| Parameter | Default | Meaning |
|---|---|---|
| `path` | `figure8` | `straight`, `circle`, `figure8` or `drawn` (selected at start-up; can be changed in the GUI) |
| `speed` | `0.20` | Forward speed during the test (m/s) |
| `radius` | `0.80` | Circle / figure-8 loop radius (m); steering = atan(L / radius) |
| `straight_length` | `2.0` | Length of the straight test (m) |
| `start_delay` | `3.0` | Countdown before moving (s); zero velocity is sent meanwhile |
| `repeat` | `false` | Loop the test until STOP |
| `autostart` | `false` | Start without pressing **Start** |
| `wheelbase`, `max_steer` | `0.20`, `0.4084` | Same meaning as for the controllers |
| `auto_place` | `true` | Centre the drawn path on the lab grid (display only) |
| `start_x`, `start_y` | `0.0`, `0.0` | With `auto_place:=false`: where the path starts on the lab grid, robot facing +X (display only) |
| `lab_x`, `lab_y` | `[-0.5, 8.0]`, `[-1.0, 6.0]` | Lab extents of the grid |

The commands depend only on the path, `speed`, `wheelbase` and `max_steer`; placement never changes them. Each command is `v = speed`, `δ = atan(L·κ)` clipped to `max_steer`, and `ω = v·tan(δ)/L`. Integrating the published commands was checked to reproduce each path: the circle and figure-8 return to the start, and the straight ends at 2.00 m.

### Launch file arguments: `hw_tracking.launch.py`

| Argument | Default | Values | Effect |
|---|---|---|---|
| `controller` | `nmpc` | `pid`, `lqr`, `mpc`, `nmpc` | Which tracker to start |
| `builder` | `true` | `true`, `false` | Start `interactive_builder` |
| `plotter` | `true` | `true`, `false` | Start `live_plotter` |
| `track_radius` | `0.0` | metres | `0` = draw a path; above 0 = capsule track with this curve radius, no drawing window |
| `track_straight`, `track_x`, `track_y`, `start_delay` | `3.0`, `3.75`, `2.5`, `5.0` | m, m, m, s | Capsule straight length and centre; warning before moving |
| `guard`, `approach` | `false`, `true` | `true`, `false` | Drift guard (extension); drive to the path start first |
| `laps` | `1` | number | Laps on a closed path; `0` = until stopped |
| `view` | `behind` | `behind`, `topdown` | Canvas orientation of both GUIs |
| `rezero_odom` | `false` | `true`, `false` | `false`: absolute MoCap frame. `true`: robot's start pose becomes (0,0). Forwarded to the controller, the builder and the plotter |
| `run_duration` | `0.0` | seconds | Controller time limit; `0` = until the path end or Ctrl-C |
| `obstacles` | `''` (none) | `"[x, y, r, ...]"` | Fixed cylinders, flat list, metres, PhaseSpace frame |
| `obstacle_markers` | `''` (none) | `"[(0,1,2,3),(4,5,6,7)]"` | 4 marker ids on the rim of each cylinder; centre and radius come live from them |
| `robot_radius`, `safety_margin` | `0.20`, `0.10` | metres | Clearance from the tracked point to a cylinder edge = their sum |
| `cbf_alpha` | `0.6` | 1/s | CBF gain; lower slows down earlier |
| `cbf_max_steer` | `0.0` | rad | Steering limit the CBF plans with; `0` = `max_steer`. Lower it if the robot's real tightest turn is wider |
| `lab_x`, `lab_y` | `''` | `"[min, max]"` | Lab extents for both GUIs; empty = `[-0.5, 8.0]` and `[-1.0, 6.0]` |
| `ref_offset` | `0.10` | metres | Marker midpoint ahead of the rear axle; forwarded to the controller |
| `robot_ns` | `limo1` | name, or `none` | Namespace of the LIMO's topics: the controller's `/cmd_vel` → `/<robot_ns>/cmd_vel`, `/imu` → `/<robot_ns>/imu`; `none` = no namespace |

### Node parameters (`--ros-args -p name:=value`)

**`phasespace_bridge`**

| Parameter | Default | Meaning |
|---|---|---|
| `front_id` | `0` | Index of the **front** marker in `msg.markers` |
| `rear_id` | `1` | Index of the **rear** marker. Heading = rear → front |
| `obstacles` | none | Flat list `[x, y, r, ...]` in metres, PhaseSpace frame: `-p "obstacles:=[1.97, 0.81, 0.35]"` |
| `marker_baseline` | `0` | Distance between the robot's two markers (m); `0` learns it from the first frames, printed at startup |
| `max_baseline_error` | `0.03` | A frame further than this from the marker distance is dropped (m) |
| `obstacle_markers` | none | 4 rim markers per cylinder. With `ros2 launch`, write `obstacle_markers:="[(0,1,2,3),(4,5,6,7)]"`. With `ros2 run`, ROS's parser rejects bare tuples, so use a flat list `-p "obstacle_markers:=[0,1,2,3,4,5,6,7]"` or quote it `-p "obstacle_markers:='[(0,1,2,3),(4,5,6,7)]'"` |

Frames where a marker is missing, has `cond <= 0`, or sits at exactly (0, 0, 0) are skipped.

**All four controllers** (`pid_tracker`, `lqr_tracker`, `mpc_tracker`, `nmpc_tracker`)

| Parameter | Default | Meaning |
|---|---|---|
| `v_nominal` | `0.40` | Cruise speed (m/s); reduced automatically in curves |
| `robot_radius`, `safety_margin` | `0.20`, `0.10` | CBF clearance from the tracked point to a cylinder edge = their sum (m) |
| `cbf_alpha`, `cbf_max_steer` | `0.6`, `0.0` | CBF gain (1/s); steering limit it plans with (rad, `0` = `max_steer`) |
| `obstacles` | none | Extra fixed cylinders for this controller, added to `/hardware_obstacles` |
| `wheelbase` | `0.20` | LIMO wheelbase L (m) |
| `ref_offset` | `0.10` | Distance of the marker midpoint **ahead of the rear axle** (m). Measure it on your robot: 0.10 assumes the midpoint is halfway between the axles |
| `max_steer` | `0.4084` | Steering limit (rad, 23.4°), the centre angle the LIMO driver allows |
| `rezero_odom` | `false` | Absolute PhaseSpace frame (`false`) or path relative to the start pose (`true`) |
| `run_duration` | `0.0` | Time limit in s; `0` = no limit |
| `log_dir`, `path_name` | `evaluation`, `drawn` | Where each run's folder goes, and the label in its name |
| `laps`, `exit_on_finish` | `1`, `false` | Laps on a closed path (`0` = until stopped); leave when the run ends |
| `guard`, `approach` | `false`, `true` | Slow, rejoin and stop on drift (extension); drive to the path start first |
| `approach_radius`, `approach_speed` | `0.60`, `0.25` | Turning radius (m) and speed (m/s) on the way to the path |
| `join_tol`, `join_heading_deg` | `0.15`, `30.0` | Closer than this to the start: no approach needed |
| `slow_cte`, `rejoin_cte`, `abort_cte`, `max_rejoins` | `0.15`, `0.35`, `1.00`, `3` | Guard levels (m), with `guard:=true` |
| `odom_timeout`, `odom_abort` | `0.5`, `3.0` | Odometry lost: zero command; with `guard:=true`, stop the run (s) |
| `guard_lab_x`, `guard_lab_y` | `[-0.5, 8.0]`, `[-1.0, 6.0]` | With `guard:=true`, the robot stops outside this area |
| `profile` | `figure8` | Only used in the CSV file name on hardware |

**Controller-specific tuning**

| Controller | Parameter | Default | Meaning |
|---|---|---|---|
| PID | `kp_cte`, `ki_cte`, `kd_cte` | `1.8`, `0.05`, `0.45` | PID gains on cross-track error |
| PID | `kp_heading` | `1.15` | P gain on heading error |
| PID | `lookahead_pts` | `10` | Look-ahead (waypoints) for heading error |
| LQR | `q_cte`, `q_heading` | `450`, `35` | State weights |
| LQR | `r_steer` | `0.5` | Steering weight |
| LQR | `lookahead_pts` | `8` | Look-ahead (waypoints) |
| LTV-MPC | `horizon_steps`, `mpc_dt` | `16`, `0.04` | Horizon length and step (s) |
| LTV-MPC | `q_cte`, `q_dcte`, `q_heading` | `520`, `35`, `42` | State weights |
| LTV-MPC | `r_steer`, `rd_steer` | `0.28`, `0.45` | Steering and steering-change weights |
| LTV-MPC | `lookahead_pts` | `9` | Look-ahead (waypoints) |
| NMPC | `horizon_steps`, `mpc_dt` | `10`, `0.08` | Horizon length and step (s) |
| NMPC | `q_cte`, `q_heading` | `40`, `8` | Error weights |
| NMPC | `rd_steer` | `2.5` | Steering slew weight |
| NMPC | `r_steer` | `0.8` | Reserved; not used in the cost |

**`interactive_builder`, `live_plotter` and `limo_test_suite`**

All three show the same canvas. `view:=behind`: +X (forward) up, +Y (left) left. `view:=topdown`: +X right, +Y up. Use the same value in all of them. Only the display changes: `/reference_path` always carries real X, Y, and the test suite shows a drawn path where it was drawn.

| Parameter | Default | Meaning |
|---|---|---|
| `rezero_odom` | `false` | Draw in the same frame as the controller. Must match the controller's value. |
| `lab_x` | `[-0.5, 8.0]` | X extent of the canvas (m): the lab floor covered by PhaseSpace |
| `lab_y` | `[-1.0, 6.0]` | Y extent of the canvas (m), e.g. `-p lab_y:="[-1.0, 5.0]"` |
| `view` | `behind` | Canvas orientation: `behind` or `topdown` |

### Topics

| Topic | Type | From → To |
|---|---|---|
| `/phasespace/markers` | `phasespace_msgs/Markers` | PhaseSpace driver → bridge |
| `/hardware_odom` | `nav_msgs/Odometry` | bridge → controllers, builder, plotter |
| `/hardware_obstacles` | `std_msgs/Float32MultiArray` (latched, flat `[x, y, r, ...]`) | bridge → controllers (CBF), builder, plotter |
| `/reference_path` | `nav_msgs/Path` | builder → controllers, plotter |
| `/reference_path_viz` | `nav_msgs/Path` | controllers → RViz (every 5th point) |
| `/<robot_ns>/cmd_vel` (e.g. `/limo1/cmd_vel`) | `geometry_msgs/Twist` | controller / test suite → LIMO (nodes publish `/cmd_vel`, remapped by the launch files) |
| `/<robot_ns>/imu` | `sensor_msgs/Imu` | LIMO → PID/LQR/MPC (unused) |
| `/run_status` | `std_msgs/String` (JSON, 5 Hz) | controller → live view, lap test |
| `/run_kill` | `std_msgs/Bool` (latched) | live view → controllers, lap test |

## GUIs

All three windows (builder, live plotter, test suite) are styled for the lab projector:
*   **thick red X and Y axes** through (0, 0), labelled **+X** and **+Y**
*   a dark 1 m grid with a light 0.5 m grid, and large tick labels
*   the lab floor as the canvas: X from −0.5 to 8 m and Y from −1 to 6 m by default (`lab_x` / `lab_y`)

The builder and the plotter show the top-down lab map (+X right, +Y up). The test suite shows the view from behind the robot (+X up, +Y left). Ctrl-C (also from `ros2 launch`) closes the windows cleanly.

Builder and plotter elements:

| Element | Look |
|---|---|
| Robot | Grey rectangle (LIMO footprint) with a **black arrow from M1 (blue, rear) to M0 (red, front)** |
| Obstacle | Filled red circle, from `/hardware_obstacles` |
| Wall | Dashed black edge (obstacle radius ≥ 100 m) |
| Status text (top left) | Robot position, heading in degrees, and which frame is used |
| Builder only | Blue drawn path (green after **Smooth Path**); buttons **Smooth Path**, **Clear**, **Publish** |
| Plotter only | Green dashed reference path; orange live trail (cleared when a new path is published); **KILL / RE-ARM** button; error chart with the guard levels; steering chart with the limit and what the curve needs; controller, phase, lap and steering use |

The obstacles come from the bridge, so the map the bridge uses is what both windows show and what the CBF avoids. There are none by default. Obstacles appear once the first odometry message has arrived.

## Simulation demo

The four-panel Gazebo demo lives in the simulation repo: see `../Ackerman_Drive/demo/README.md`. It runs these same controllers. `obstacle_mode:=constraint` on `mpc_tracker` and `nmpc_tracker` uses obstacles as MPC constraints instead of the CBF; the default stays `cbf`.

## MATLAB version

The `matlab/` folder (workspace root) contains a MATLAB implementation of the same pipeline:
*   free-hand path drawing on the lab grid, with smoothing
*   the four controllers (PID, LQR, LTV-MPC, NMPC), with the same models and tuning
*   simulation, metrics, plots and animation
*   `run_on_limo.m`, which runs a MATLAB controller on the real robot through ROS 2, using the Python `phasespace_bridge` for odometry

It needs only base MATLAB; the Control System Toolbox is optional for `lqr`, and the ROS Toolbox is needed for hardware runs. See [`matlab/README.md`](../matlab/README.md) for requirements and a full explanation.

## Documentation
*   `docs/Lab_Guide.pdf` (source `LAB_GUIDE.md`): the lab day, step by step.
*   `docs/Design_Report.pdf` (`DESIGN_REPORT.md`): every design choice, why, the metrics, and likely questions.
*   `docs/Code_Guide_Controllers.pdf` (`CODE_GUIDE_controllers.md`): the four controllers, the shared model and obstacle handling, function by function.
*   `docs/Code_Guide_System.pdf` (`CODE_GUIDE_system.md`): bridge, run manager, way to the start, suites, reports and tools, function by function.
*   `docs/Theory_Explanation.pdf`: kinematics, error dynamics, the maths of every controller, obstacles and reaching the start.
*   `docs/Code_Explanation.pdf`: older full listing (before 2026-11-05), kept for its appendix on how the simulation package differs; the two code guides replace it.
*   Rebuild the PDFs: `docs/build_docs.sh` (the guides, via pandoc and `docs/tex/`) and `latexmk -xelatex docs/Theory_Explanation.tex`. Fonts: Fira Sans, Fira Mono, Fira Math from TeX Live.
*   `CHANGELOG.md` (workspace root): what changed, from what, and why.
*   Rebuild either PDF with `./build_pdf.sh <file>.tex` from `docs/`.
