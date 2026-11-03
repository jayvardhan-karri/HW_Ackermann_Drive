# LIMO MoCap Control Suite

A hardware-agnostic, off-board trajectory tracking suite for the AgileX LIMO. This package bridges PhaseSpace Motion Capture (MoCap) data to standard ROS 2 Odometry, allowing you to draw trajectories interactively and track them using highly-tuned control algorithms (PID, LQR, LTV-MPC, and NMPC) without modifying the robot's onboard software.

> [!IMPORTANT]
> **Robot heading = the vector from marker 1 to marker 0.**
> Marker **0** (`front_id`) must be on the **front** of the LIMO and marker **1** (`rear_id`) on the **rear**, on the robot's centre line. The bridge computes `yaw = atan2(y0 - y1, x0 - x1)`, and the position is the midpoint of the two markers. The GUIs draw the robot with this convention: a black arrow from the **blue dot (M1, rear)** to the **red dot (M0, front)**. If the markers are mounted the other way round, swap them with `-p front_id:=1 -p rear_id:=0` instead of re-mounting.

## Project Architecture & File Breakdown

The package is modular, separating trajectory generation, state estimation, safety, and control math:

*   **`phasespace_bridge.py`**: The Hardware Abstraction Layer. Subscribes to `/phasespace/markers` (`phasespace_msgs/Markers`), computes the LIMO's centroid and yaw (marker 1 → marker 0), and publishes `/hardware_odom`. It also generates a synthetic `/hardware_scan` from the obstacle map and publishes that map on `/hardware_obstacles`, so the GUIs draw the same obstacles.
*   **`interactive_builder.py`**: A Matplotlib-based GUI. Click and drag to draw a path, apply B-spline smoothing, and publish it to `/reference_path`. It shows the live robot pose and heading and the obstacles while you draw.
*   **`live_plotter_node.py`**: A Matplotlib-based live view of a run: reference path, robot trail, robot glyph with heading, and obstacles.
*   **`limo_test_suite.py`**: Open-loop actuator test with a GUI. It plays a test path (straight, left circle, figure-8, or a drawn path) as `/cmd_vel` commands and shows, from behind the robot, where the LIMO should go and how its front wheels should be steered. No MoCap or feedback needed; the robot can sit with its wheels in the air.
*   **`ros_compat.py`**: Foxy / Humble shim (`init_ros`), so the same code runs on both.
*   **`plot_helpers.py`**: Shared drawing code for the GUIs: lab grid with red axes for the projector, robot glyph, obstacles, controller frame, Ctrl-C handling.
*   **`trajectory_subscriber.py`**: A mathematical utility class used by all controllers. It subscribes to `/reference_path` and calculates local derivatives (curvature $\kappa$ and reference velocity $v_{\text{ref}}$). It also detects whether the path is a closed loop or an open path.
*   **`lidar_safety.py`**: The Artificial Potential Field (APF) safety layer, used by all four controllers. Scales velocity down near obstacles and applies repulsive steering vectors based on laser scan data.
*   **Controllers** (subscribe to `/hardware_odom` and `/hardware_scan`):
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

## Build Instructions

Ensure you are in the root of your hardware workspace (e.g., `HW_Ackermann_Drive`):

```bash
colcon build --packages-select limo_mocap_suite
source install/setup.bash
```

## Usage

Because the control stack runs off-board, the base station and the LIMO must share the same `ROS_DOMAIN_ID`.

> [!IMPORTANT]
> **The LIMO's topics are namespaced.** On the lab robot, `ros2 topic list` shows `/limo1/cmd_vel`, `/limo1/imu`, and so on. The nodes publish to `/cmd_vel`, so they must be remapped, otherwise the robot never receives a command and simply does not move. Both launch files do this through `robot_ns` (default **`limo1`**):
> *   `robot_ns:=limo2` for another robot
> *   `robot_ns:=none` for a robot without a namespace (plain `/cmd_vel`)
>
> With `ros2 run`, add the remap yourself: `--ros-args -r /cmd_vel:=/limo1/cmd_vel` (and `-r /imu:=/limo1/imu` for PID/LQR/MPC; it is unused, so optional). In MATLAB set `P.ros.cmd_topic`. Only `/cmd_vel` and `/imu` belong to the robot; `/hardware_odom`, `/hardware_scan`, `/hardware_obstacles` and `/reference_path` come from this package and are never remapped.
>
> Check: `ros2 node info /nmpc_tracker_node` must list `/limo1/cmd_vel` under *Publishers*.

### Quick start (one terminal)

```bash
source install/setup.bash
ros2 launch limo_mocap_suite hw_tracking.launch.py controller:=nmpc rezero_odom:=false   # /cmd_vel -> /limo1/cmd_vel (robot_ns default)
```

This starts the bridge, the controller, the builder and the live plotter. Draw a path in the builder, click **Smooth Path**, then **Publish**. With the lab obstacle:
```bash
ros2 launch limo_mocap_suite hw_tracking.launch.py controller:=nmpc rezero_odom:=false \
    obstacles:="[1.970681, 0.80659, 0.35]"
```

### Step by step (separate terminals)

**1. PhaseSpace bridge**
```bash
ros2 run limo_mocap_suite phasespace_bridge --ros-args -p "obstacles:=[1.970681, 0.80659, 0.35]"
```

**2. A controller.** It idles until a path arrives.
```bash
ros2 run limo_mocap_suite nmpc_tracker --ros-args -p rezero_odom:=false -r /cmd_vel:=/limo1/cmd_vel
# alternatives: pid_tracker, lqr_tracker, mpc_tracker (same -r remap; add -r /imu:=/limo1/imu if you like)
```

**3. Path builder.**
```bash
ros2 run limo_mocap_suite interactive_builder --ros-args -p rezero_odom:=false
```
*   Click and drag on the canvas to draw your route.
*   Click **Smooth Path** to apply B-spline curvature smoothing.
*   Click **Publish**. The idling controller locks on and begins tracking.
*   A stroke that ends where it started is a **closed loop** (driven repeatedly). Any other stroke is an **open path**, and the controller stops by itself at its end.

**4. Live plotter** (optional).
```bash
ros2 run limo_mocap_suite live_plotter --ros-args -p rezero_odom:=false
```

> [!NOTE]
> **Use the same `rezero_odom` value everywhere** (controller, builder, live plotter).
> *   `false` (recommended in the lab): everything is in the absolute PhaseSpace frame, the same frame as the obstacle map.
> *   `true` (default): coordinates are relative to the robot's pose when each node received its first odometry. The canvas origin is then the robot, facing +X. Start all nodes together, before the robot moves, so they agree on that pose.

### LIMO test suite (wheels in the air, no feedback)

Checks that the LIMO's wheels do what `/cmd_vel` says, before any closed-loop run. Lift the robot so its wheels spin freely (or put it on the floor: it just drives the path blind), stand **behind** it, and compare with the screen.

```bash
ros2 launch limo_mocap_suite limo_test.launch.py                       # sends to /limo1/cmd_vel
ros2 launch limo_mocap_suite limo_test.launch.py path:=circle speed:=0.15 robot_ns:=limo1
# or without launch (remap yourself), starting straight away:
ros2 run limo_mocap_suite limo_test_suite --ros-args -r /cmd_vel:=/limo1/cmd_vel -p path:=circle -p speed:=0.15 -p autostart:=true
```
`limo_test.launch.py` arguments: `robot_ns` (default `limo1`, `none` = plain `/cmd_vel`), `path` (default `figure8`), `speed` (default `0.20`).

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
| `wheelbase`, `max_steer` | `0.20`, `0.48869` | Same meaning as for the controllers |
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
| `use_synthetic_scan` | `true` | `true`, `false` | Bridge publishes the ray-cast `/hardware_scan` |
| `rezero_odom` | `true` | `true`, `false` | Forwarded to the controller, the builder and the plotter |
| `run_duration` | `0.0` | seconds | Controller time limit; `0` = until the path end or Ctrl-C |
| `obstacles` | `''` | `"[x, y, r, ...]"` | Bridge obstacle map; empty = the bridge default (simulation arena) |
| `lab_x`, `lab_y` | `''` | `"[min, max]"` | Lab extents for both GUIs; empty = `[-0.5, 8.0]` and `[-1.0, 6.0]` |
| `robot_ns` | `limo1` | name, or `none` | Namespace of the LIMO's topics: the controller's `/cmd_vel` → `/<robot_ns>/cmd_vel`, `/imu` → `/<robot_ns>/imu`; `none` = no namespace |

### Node parameters (`--ros-args -p name:=value`)

**`phasespace_bridge`**

| Parameter | Default | Meaning |
|---|---|---|
| `front_id` | `0` | Index of the **front** marker in `msg.markers` |
| `rear_id` | `1` | Index of the **rear** marker. Heading = rear → front |
| `use_synthetic_scan` | `true` | Publish the ray-cast `/hardware_scan` |
| `obstacles` | simulation arena | Flat list `[x, y, r, x, y, r, ...]` in metres, PhaseSpace frame. Radius ≥ 100 m is drawn as a wall. The default is 4 pillars at (±2.5, ±1.5), r 0.15, plus walls at ±5 m. |

Frames where a marker is missing, has `cond <= 0`, or sits at exactly (0, 0, 0) are skipped.

**All four controllers** (`pid_tracker`, `lqr_tracker`, `mpc_tracker`, `nmpc_tracker`)

| Parameter | Default | Meaning |
|---|---|---|
| `v_nominal` | `0.40` | Cruise speed (m/s); reduced automatically in curves |
| `wheelbase` | `0.20` | LIMO wheelbase L (m) |
| `max_steer` | `0.48869` | Steering limit (rad, 28°) |
| `rezero_odom` | `true` | Path relative to the start pose (`true`) or absolute PhaseSpace frame (`false`) |
| `run_duration` | `45.0` (PID), `25.0` (others) | Time limit in s; `0` = no limit. An open path also ends the run. |
| `log_dir` | `evaluation` | Folder (relative to the launch directory) for `<ctrl>_<profile>_telemetry.csv` |
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

**`interactive_builder` and `live_plotter`**

| Parameter | Default | Meaning |
|---|---|---|
| `rezero_odom` | `true` | Draw in the same frame as the controller. Must match the controller's value. |
| `lab_x` | `[-0.5, 8.0]` | X extent of the canvas (m): the lab floor covered by PhaseSpace |
| `lab_y` | `[-1.0, 6.0]` | Y extent of the canvas (m), e.g. `-p lab_y:="[-1.0, 5.0]"` |

### Topics

| Topic | Type | From → To |
|---|---|---|
| `/phasespace/markers` | `phasespace_msgs/Markers` | PhaseSpace driver → bridge |
| `/hardware_odom` | `nav_msgs/Odometry` | bridge → controllers, builder, plotter |
| `/hardware_scan` | `sensor_msgs/LaserScan` | bridge → controllers |
| `/hardware_obstacles` | `std_msgs/Float32MultiArray` (latched, flat `[x, y, r, ...]`) | bridge → builder, plotter |
| `/reference_path` | `nav_msgs/Path` | builder → controllers, plotter |
| `/reference_path_viz` | `nav_msgs/Path` | controllers → RViz (every 5th point) |
| `/<robot_ns>/cmd_vel` (e.g. `/limo1/cmd_vel`) | `geometry_msgs/Twist` | controller / test suite → LIMO (nodes publish `/cmd_vel`, remapped by the launch files) |
| `/<robot_ns>/imu` | `sensor_msgs/Imu` | LIMO → PID/LQR/MPC (unused) |

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
| Plotter only | Green dashed reference path; orange live trail (cleared when a new path is published) |

The obstacles come from the bridge, so whatever map the bridge uses (default or `obstacles:=...`) is what both windows show, and what the synthetic LiDAR sees. Obstacles appear once the first odometry message has arrived.

## MATLAB version

The `matlab/` folder (workspace root) contains a MATLAB implementation of the same pipeline:
*   free-hand path drawing on the lab grid, with smoothing
*   the four controllers (PID, LQR, LTV-MPC, NMPC), with the same models and tuning
*   simulation, metrics, plots and animation
*   `run_on_limo.m`, which runs a MATLAB controller on the real robot through ROS 2, using the Python `phasespace_bridge` for odometry

It needs only base MATLAB; the Control System Toolbox is optional for `lqr`, and the ROS Toolbox is needed for hardware runs. See [`matlab/README.md`](../matlab/README.md) for requirements and a full explanation.

## Documentation
*   `docs/Theory_Explanation.pdf`: kinematics, error dynamics and the maths of every controller.
*   `docs/Code_Explanation.pdf`: every file and function, plus how the simulation package differs.
*   `CHANGELOG.md` (workspace root): what changed, from what, and why.
*   Rebuild either PDF with `./build_pdf.sh <file>.tex` from `docs/`.
