# Code guide: the system around the controllers

This guide explains what the code around the four controllers does and how: the mocap bridge, the run manager, the planner that brings the robot onto the track, the lap tests, the report, the logs and the launch files. The controllers, the CBF and the MPC obstacle constraints are in `docs/CODE_GUIDE_controllers.md`. The reasons behind the choices are in `docs/DESIGN_REPORT.md`.

Paths are relative to `src/limo_mocap_suite/` unless they start with `tools/` or `docs/`. Python modules are in `limo_mocap_suite/` and launch files in `launch/`. A number in the Line column, or in parentheses after a name, is the line of the definition in that file. The tags `[CORE]`, `[ROS-IO]`, `[UI]`, `[LOG]`, `[LIFE]` and `[UTIL]` in the code comments mark what a function is for.

## 1. The big picture

### 1.1 Data flow

```
 PhaseSpace cameras + 2 LED markers on the LIMO (id 0 front, id 1 rear)
        |  /phasespace/markers  (phasespace_msgs/Markers, queue depth 1)
        v
 phasespace_bridge  (PoseGate drops ghost frames)
        |-- /hardware_odom      nav_msgs/Odometry, midpoint, yaw rear->front
        '-- /hardware_obstacles Float32MultiArray [x,y,r,...], latched

 path source (one of):
   interactive_builder  (hand-drawn)             -> /reference_path
   track_publisher      (hw_tracking track_radius>0) -> /reference_path
   run_laps             (lap suites)             -> /reference_path
                                                    (re-sent every 3 s)

 controller node (pid | lqr | mpc | nmpc _tracker)
   subscribes /hardware_odom, /reference_path, /imu, /hardware_obstacles
   owns a RunManager: WAIT_PATH -> APPROACH -> TRACK -> WAIT_PATH
     publishes  /cmd_vel   (Twist: v, omega = v/L tan(steer))
     publishes  /run_status (JSON, 5 Hz), /reference_path_viz
     subscribes /run_kill  (Bool, latched)
     writes     <run folder>/telemetry.csv and summary.json
        |  the launch file remaps /cmd_vel -> /limo1/cmd_vel
        v
 LIMO (ROS 2 driver, namespace limo1)

 live_plotter: subscribes path, odometry, obstacles, status;
               publishes /run_kill (KILL button) and a zero /cmd_vel
 run_laps    : starts each controller as a child process, publishes the
               path, watches status, kill and odometry, then runs report_lap
```

The `/imu` subscription exists in PID, LQR and LTV-MPC but its callback is empty. NMPC has no IMU subscription.

The topics of the robot (`/cmd_vel`, `/imu`) are remapped into the namespace (`robot_topics.py:25`, default namespace `limo1`, `robot_topics.py:26`). The topics made by this package (`/hardware_odom`, `/hardware_obstacles`, `/reference_path`, `/reference_path_viz`, `/run_status`, `/run_kill`) are never remapped.

### 1.2 Frames and the lab

| Item | Value | Where |
|---------|---------------------------------|------------------|
| Mocap frame | ROS right-handed: +x forward, +y left, yaw counter-clockwise. The bridge mirrors and rotates nothing | `phasespace_bridge.py` header |
| Robot markers | id 0 front, id 1 rear (`front_id`, `rear_id`) | `phasespace_bridge.py:143-144` |
| Pose | midpoint of the markers; yaw = `atan2(fy-ry, fx-rx)` | `PoseGate.accept`, `:97` |
| Lab area | x from -0.5 to 7.0 m, y from -0.5 to 3.0 m | `track_tools.py:40-41` |
| Track | $R = 1.0$ m, straights 4.0 m, centre (3.25, 1.25) | `track_tools.py:42-45` |
| Tracked point | marker midpoint, 0.10 m ahead of the rear axle | `ref_offset` |
| Frame labels | `odom` and `base_footprint` are labels only; no tf is used | `phasespace_bridge.py` |
| `rezero_odom` | false by default in all launch files: all nodes use the absolute mocap frame | launch files |

The track spans x 0.25 to 6.25 m and y 0.25 to 2.25 m, which is 0.75 m from every lab edge. The GUIs can rotate the picture ("behind" view: +X up, +Y left), but all ROS messages carry the real X, Y (`plot_helpers.to_view` and `from_view`).

## 2. Which launch file for what

Order in the lab: `hw_check` first, then `limo_test` if the connection is in doubt, then a lap file.

| Launch file | Use it for | Asked when missing |
|------------|-----------------------|-------------------------|
| `hw_check.launch.py` | Lab check; with `drive:=true` also a slow test drive | `drive` (false) |
| `limo_test.launch.py` | Open-loop play of a path as `/cmd_vel`; no feedback | `path` (figure8), `speed` (0.20) |
| `hw_tracking.launch.py` | One controller on a drawn path or the R1 track | `controller` (nmpc), `track_radius` (0.0) |
| `record_lap.launch.py` | One controller, laps on R1, then a report | `controller`, `laps`, `start_side`, `sim` |
| `compare_lap.launch.py` | All four controllers in turn on R1, one report | `controllers`, `laps`, `buffer_time`, `start_side`, `sim` |
| `obstacle_lap.launch.py` | Add-on: R1 with two virtual cylinders | `compare_modes`, `controllers`, `laps`, `start_side`, `sim` |
| `r05_lap.launch.py` | Add-on: like compare, but $R = 0.5$ m | `controllers`, `laps`, `start_side`, `sim` |
| `run_laps.launch.py` | The engine behind the four lap files | nothing |

What each file starts:

- `hw_check.launch.py`: `phasespace_bridge` (unless `bridge:=false`) and `hw_check`. The whole launch ends when `hw_check` exits.
- `limo_test.launch.py`: `limo_test_suite`.
- `hw_tracking.launch.py`: `phasespace_bridge`, `<ctrl>_tracker`, `live_plotter`, `interactive_builder` (only when drawing) and `track_publisher` (only when `track_radius` is above 0).
- `record_lap.launch.py`: includes `run_laps.launch.py` with one controller, `track_radii:=1.0` and `session_name:=record_<ctrl>`. Default laps: 3.
- `compare_lap.launch.py`: includes `run_laps.launch.py` with `track_radii:=1.0`, `session_name:=compare`. Defaults: all four controllers, 3 laps, `buffer_time` 10 s.
- `obstacle_lap.launch.py`: R1 with two cylinders of 0.60 m diameter, published by the bridge (or `sim_limo`). It includes `run_laps.launch.py` with `obstacles`, `start_at:=begin` and `session_name:=obstacle_lap`. With `compare_modes:=true` the controllers become `mpc,nmpc,mpc:constraint,nmpc:constraint` and the session `obstacle_modes`. The argument `controllers` is then skipped.
- `r05_lap.launch.py`: includes `run_laps.launch.py` with `track_radii:=0.5` and `session_name:=r05_lap`. The track spans x 0.75 to 5.75 m and y 0.75 to 1.75 m. A radius of 0.5 m is close to the steering limit (minimum radius 0.46 m).
- `run_laps.launch.py`: `phasespace_bridge` (or `sim_limo` if `sim:=true`), `live_plotter`, `run_laps`. You can also call it directly, for example with `track_radii:=1.0,0.5,0.4`.

**Prompts** (`launch_prompts.py`).

- A value given on the command line (`name:=value`) is used and never asked.
- A missing value is asked in the terminal with numbered choices; Enter takes the default.
- Without a terminal the default is used. A required value without default raises an error.
- After the questions a heading lists every setting and its source: "command line", "chosen" or "default".
- `ask_first(...)` is placed first in the `LaunchDescription` and writes the answers into the launch configurations, so included launch files see them as if typed on the command line. This is why `obstacle_mode:=constraint`, `start_side`, `sim` and `view` reach `run_laps.launch.py` without being listed in `launch_arguments`.
- Not asked, set only on the command line: `v_nominal`, `direction`, `start_at`, `guard`, `view`, `robot_ns`, `obstacle_mode`, `obstacles`, `approach_*`, `log_dir`.

**Defaults of the lap engine** (`run_laps.launch.py:106-141`, repeated in `run_laps.py:79-90`):

| Argument | Default | Meaning |
|--------------------|-----------|------------------------------|
| `controllers` | `pid,lqr,mpc,nmpc` | Order of the runs; `mpc:constraint` runs that controller with `obstacle_mode` constraint |
| `laps` | 3 | Laps per controller |
| `guard` | false | Drift guard off |
| `v_nominal` | 0.30 m/s | Cruise speed (the controllers' own default is 0.40) |
| `track_radii` | 1.0 | Comma list; one folder `R<r>` per radius |
| `start_side` | auto | Top straight, or bottom if the robot is below the track centre line |
| `direction` | ccw | Left turns; `cw` right turns |
| `start_at` | middle | Middle of the straight; `begin` where it begins |
| `buffer_time` | 10.0 s | Robot held at zero between two controllers |
| `start_delay` | 5.0 s | Warning before the robot moves |
| `approach_radius` | 0.60 m | Turning radius of the way to the start |
| `approach_speed` | 0.25 m/s | Speed on the way |
| `approach_spill` | 0.30 m | Allowed overshoot past the lab edge |
| `slow_cte`, `rejoin_cte`, `abort_cte` | 0.15, 0.35, 1.00 m | Guard levels (only with `guard:=true`) |
| `obstacle_mode` | cbf | `constraint` puts obstacles inside the MPC solvers |
| `log_dir` | empty | Means `<workspace>/evaluation` |
| `robot_ns` | limo1 | `none` means no namespace |

## 3. File by file

### 3.1 `phasespace_bridge.py`

Console script `phasespace_bridge`. It turns two PhaseSpace markers into `/hardware_odom` and publishes the obstacle map. The callback `marker_cb` (`:210`) does:

1. Keep only markers with `cond > 0` that are not exactly at (0, 0, 0) (PhaseSpace sends that for "not seen").
2. Update obstacles that come from rim markers, even if the robot is lost.
3. If the front or rear marker is missing, return. Nothing is published, and the controllers then hold zero.
4. `PoseGate.accept(...)` drops ghost frames (below). A dropped frame publishes nothing.
5. Take the finite-difference world velocity, project it on the previous heading and low-pass it with time constant `V_FILTER_TAU` = 0.1 s (`:58`). It is used only as `twist.linear.x` for telemetry (`v_actual`).
6. Publish `Odometry` with stamp = now and frames `odom` and `base_footprint`.

**PoseGate rules** (constants `:52-58`):

| Rule | Value | Effect |
|------------|-----------------------------|-------------------|
| Learn marker distance | median of the first 20 frames (`BASELINE_FRAMES`), or fixed by `marker_baseline` | Nothing is published while learning |
| Wrong marker distance | differs from the baseline by more than 0.03 m (`max_baseline_error`) | Frame dropped |
| Impossible jump | moved more than $0.05 + 1.5\,dt$ m, or turned more than $0.25 + 3.0\,dt$ rad | Frame dropped |
| Re-lock | the 10th jump in a row (`RELOCK_FRAMES`) | Accepted: the robot was really moved |

A marker that jumps (reflection, swapped ids) would swing the heading and slam the steering. Without odometry the controllers send zero.

**Obstacles.** The parameter `obstacles` is a flat list `[x, y, r, ...]` (none by default). The parameter `obstacle_markers`, for example `[(0,1,2,3),(4,5,6,7)]`, gives 4 rim markers per cylinder: the centre is the mean and the radius the mean distance to the centre. A circle that moves by more than 2 cm in any value is re-published, and an occluded group keeps its last circle. The list goes out on `/hardware_obstacles` with latched QoS (`TRANSIENT_LOCAL`), so late GUIs still get it. `phasespace_msgs` comes from the lab's `phasespace_bringup` workspace and must be built and sourced.

| Function | Line | What it does |
|------------------|-----|-------------------------------------|
| `PoseGate` | 68 | Decides if a frame can be believed; plain math, no ROS |
| `PoseGate.__init__` | 76 | Baseline (0 = learn), tolerance, counters |
| `PoseGate.accept` | 97 | Returns `(x, y, yaw)` or `None` |
| `PhaseSpaceBridgeNode` | 130 | The ROS node |
| `__init__` | 139 | Declares `front_id`, `rear_id`, `marker_baseline`, `max_baseline_error`, obstacles; publishers; subscriber depth 1; 5 s health timer |
| `euler_to_quat` | 193 | Yaw to quaternion |
| `marker_cb` | 210 | One mocap frame in, at most one Odometry out |
| `update_marker_obstacles` | 269 | Circle fit from rim markers; republish on change |
| `publish_obstacles` | 288 | Fixed and marker circles as a flat list |
| `report_health` | 301 | 5 s log: marker distance, dropped frames |
| `warn_missing_obstacles` | 320 | 5 s warning about marker groups never fully tracked |
| `main` | 333 | Entry point |

### 3.2 `hw_check.py`

Console script `hw_check`, normally started by `hw_check.launch.py`. It is the lab check before tracking. It finds what a desk test cannot: a mirrored frame, swapped markers, a wrong namespace, a robot that keeps driving when commands stop, and how the real steering compares with the model. It reports and suggests; it changes nothing. The report is `<log_dir>/hw_check_<stamp>/report.json`.

**Stage 1, robot still (always).** Wait up to 15 s for odometry, then record `still_time` = 5 s. Checks: rate at least `MIN_RATE` 50 Hz, longest gap at most `MAX_GAP` 0.20 s, position jitter (standard deviation) at most `MAX_POS_JITTER` 5 mm, heading jitter at most 1 deg, pose moved by more than 2 cm (WARN), and whether `/cmd_vel` has a subscriber (FAIL if none).

**Stage 2, slow drive** (`drive:=true`, speed 0.15 m/s, only if stage 1 has no FAIL). After `start_delay` = 5 s:

1. A straight of `SEGMENT_STRAIGHT` 3.0 s: speed gain (PASS from 0.8 to 1.2), trim (WARN if curvature is above 0.05 1/m), start lag and the direction of travel $\beta$. The test stops if $\beta$ is more than 60 deg from the yaw (if about 180 deg, `front_id` and `rear_id` are swapped).
2. Arcs at `ARC_STEER` 0.25 rad for `SEGMENT_ARC` 3.5 s, then full lock (`max_steer` 0.4084) for `SEGMENT_FULL` 4.0 s: left, full left, right, full right. A turn in the wrong direction stops the run ("mocap frame is mirrored").
3. Commands-stop test: drive 2 s, send nothing for 1.5 s. The robot counts as stopped if its speed in the last second is below 25 % of the command.

Between segments it sends zeros for 1 s, and the first `SETTLE` = 1.5 s of each segment is not measured. Any FAIL, KILL, lost odometry (`odom_timeout` 0.5 s) or leaving `max_range` (3.0 m from the start) stops the robot at once. Leaving `max_range` is a WARN "Incomplete", not a robot fault.

`analyse` derives: the curvature gain each way, measured over $\tan(0.25)/L$ (PASS from 0.8 to 1.2); a left-right difference above 0.1 (WARN); the `ref_offset` estimate, the mean of $\tan\beta/\kappa$ for left and right (WARN if more than 0.03 m from the configured value); the tightest radius each way against $L/\tan(\delta_{\max})$ (PASS within 0.08 m); and the suggestions `tightest_radius_m` and `real_steer_limit_rad` $= \arctan(L/\text{radius})$.

The exit code is 1 if any FAIL, else 0. The verdicts are "FAIL: do not run tracking", "Still checks done. Now run it with drive:=true", "Incomplete..." and "OK to run tracking (n warning(s) to read)".

| Function | Line | What it does |
|-------------|-------|----------------------------------------|
| `Stop` | 71 | Exception that ends the drive at once (KILL, odometry lost, bad result) |
| `OutOfRoom` | 81 | `Stop` for leaving `max_range`; reported as WARN |
| `wrap` | 90 | Angle to $(-\pi, \pi]$ |
| `measure` | 102 | From one steady stretch: speed, curvature, turn, $\beta$ |
| `analyse` | 142 | Measurements to findings and suggestions |
| `HwCheck` | 208 | The node; `run()` in the main thread, `rclpy.spin` in another |
| `__init__` | 215 | Parameters, `/cmd_vel` publisher, `/run_kill` (latched), `/hardware_odom` (depth 50) |
| `_kill` | 237 | Remembers KILL |
| `_odom` | 246 | Stores (monotonic time, x, y, yaw) |
| `take`, `latest` | 260, 267 | Samples in a time window; newest sample |
| `send` | 277 | One command like the controllers send |
| `stop` | 289 | Zeros at 50 Hz |
| `check` | 302 | Safety test every 20 ms: KILL, odometry age, distance from the origin |
| `drive_for` | 319 | Holds a command, then stops 1 s |
| `still_stage` | 338 | Stage 1 |
| `drive_stage` | 381 | Stage 2: straight, left, full left, right, full right, commands-stop |
| `run` | 450 | Runs the stages, stops the robot, then analyses and writes `report.json` |
| `main` | 506 | Spin thread plus `run`; always ends with zero commands |

`measure` sums the distance on 0.2 s steps (not every frame, because mocap noise would add up). The order in `run`, stop first and analyse afterwards, matters.

### 3.3 `run_manager.py`

Used by all four controllers. It does everything except computing the steering: reach the path start, count laps, obey KILL, protect against lost odometry, log each run and wait for the next path. It is one shared class, so all controllers are treated the same. The controller calls it once per tick:

```
tick = self.run.tick(x, y, yaw, lookahead)   # None: nothing to command
cte, herr, ref_pt, idx, dt = tick
... controller computes v, steer ...
v, steer = self.run.shape(v, steer, idx)     # guard speed factor, clip
v, steer, clear, status = self.safety.apply(...)   # CBF
self.run.command(v, steer, clear, status, solve_ms)
```

The phases are `WAIT_PATH`, `APPROACH` (only if the robot is not already on the start), `TRACK`, and back to `WAIT_PATH`.

| Behaviour | Where | Detail |
|---------------|-----------|---------------------------------|
| KILL is latched | `_kill_cb` :149 | `killed` stays true until re-arm (a `False` message). On KILL: zero Twist, run ends as `KILLED`. On re-arm `traj.new_path` is cleared, so a path sent before the kill cannot start by itself. |
| Odometry lost | `tick` :201 | Newest odometry older than `odom_timeout` (0.5 s): zero Twist, guard state `HOLD`, return None. It resumes by itself. With `guard:=true` it aborts after `odom_abort` (3.0 s). |
| No run from a stale pose | `tick` :190 | A new path starts only if `now - last_odom <= odom_timeout`. |
| Time limit | `tick` :207 | `run_duration` above 0: finish as `TIME_LIMIT` (`run_laps` sets it). |
| Lab limit | `tick` :213 | Guard only: pose outside `guard_lab_x/y` ends the run as `ABORTED`. |
| Approach deadline | `tick` :217 | Guard only: 15 s plus $3 \cdot n_{way}\cdot 0.01 / \max(v, 0.05)$ (`:476`). |
| Path too tight | `_start_run` :343 | Warning: peak curvature above $\tan(\delta_{\max})/L$. |
| Nobody listens | `_start_run` :308 | Error log "robot will not move. Is it on, on this ROS_DOMAIN_ID, and is robot_ns right?" |

The guard is off by default (`guard` False): no slowing, no rejoin, no abort on error. The controller alone decides.

**Parameters** declared by `RunManager` if the node has not declared them (`:92-100`): `log_dir` evaluation, `path_name` drawn, `run_duration` 0.0, `laps` 1, `exit_on_finish` False, `guard` False, `approach` True, `approach_radius` 0.60, `approach_speed` 0.25, `join_tol` 0.15, `join_heading_deg` 30.0, `slow_cte` 0.15, `rejoin_cte` 0.35, `abort_cte` 1.00, `max_rejoins` 3, `odom_timeout` 0.5, `odom_abort` 3.0, `obstacle_margin` 0.50, `approach_spill` 0.30, `guard_lab_x` and `guard_lab_y`. Constants (`:60-62`): `SLOW_MIN` 0.4, `PREFIX_LEN` 1.5 m, `EDGE_MARGIN` 0.25 m.

**Starting a run** (`_start_run`, `:294`). A new path arrived. Close any run in progress (`REPLACED`), copy the path as the main table, compute its spacing (median segment) and length, and set `laps` from the parameter if the path is closed, else 1 (a warning is printed if more laps were asked). Create `<log_dir>/<stamp>_<controller>_<path_name>/` and open `telemetry.csv`. If the robot is within `join_tol` (0.15 m) of the first point and within `join_heading_deg` (30 deg) of its heading, or `approach:=false`, go straight to `TRACK`. Otherwise plan the way to the start (`APPROACH`).

**The way to the start** (`approach_planner.plan_to_start`, `:195`). It is a fixed rule for closed paths, with no scores or weights: the same situation always gives the same plan.

1. **Lane.** The closest path point that is in front of the robot and runs within 90 deg of its heading ($\cos(\theta_{path} - \theta_{robot}) \geq 0$). If none is in front, any point running the same way.
2. **Merge point.** `MERGE_AHEAD` = 1.0 m further along the path from there, reached by the shortest Dubins curve of radius `approach_radius` (0.60 m).
3. **Then follow the real path to the start.** If the start comes before the merge point, merge onto the start itself.
4. **Checks.** The curve may not turn by more than `MAX_TURN` = 180 deg in total (no loop), may not leave the allowed area, and may not come within `obstacle_margin` = 0.5 m of an obstacle edge (`_allowed`). If it fails, the merge point moves `MERGE_STEP` = 0.5 m further, up to `MERGE_MAX` = 6.0 m.
5. **Fallback.** If no merge works (robot sideways near the end of the loop), take the shortest Dubins curve straight to the start, where loops are allowed. The log text starts with "no smooth merge:". If nothing is allowed, the result is None.

`_plan_way_back` (`:425`) wraps this and tries three levels of "allowed area" in order:

| Level | Allowed area |
|-------|-----------------------------------------------------|
| 1 | `EDGE_MARGIN` = 0.25 m inside the lab edges |
| 2 | The lab itself (edge 0) |
| 3 | `approach_spill` = 0.30 m past the edge, with a warning; skipped when `guard:=true` |

If no level works: for the way to the start, the run ends `ABORTED` with "cannot reach the start without going more than 0.3 m out of the lab (or through an obstacle). Robot not moved: put it at least 0.6 m inside the edges, not facing out, then start again". With the guard on and no way, the run ends `ABORTED` with "no drivable way back inside the lab". A rejoin without guard and without any allowed way falls back to the shortest curve, with a warning.

The table the controller tracks is: the curve at `approach_speed` (0.25 m/s), then the lead-in stretch of the path up to the start, then `PREFIX_LEN` (1.5 m) of the real path, so the MPC horizon sees what comes next. The controller tracks it as a normal open path. The phase is `APPROACH`.

**Join** (`tick` :222, `_join_path` :490). In `APPROACH`, when the robot's closest index reaches the end of the way (`join_at`), the main table is loaded and the phase becomes `TRACK`. `approach_time_s` is stored.

**Lap counting** (`_update_progress`, `:514`). It counts forward progress: $d$ is the index change since the last tick, wrapped to half the table, and only $d > 0$ is added to `progress`. A lap is done when `progress` reaches $\text{lap}\cdot n$ ($n$ = path points). Standing or wobbling at the start line cannot count as a lap, and a part skipped after a rejoin counts as driven. `_end_lap` stores the lap statistics. The run is `FINISHED` after `laps` laps ("n laps done"). On an open path `_update_progress` does nothing, and the run ends at `goal_reached` ("end of path").

**Guard** (`_update_guard`, `:562`, only with `guard:=true`):

| Level | Condition | Action |
|-----|---------------------------|---------------------------|
| `SLOW` | $\lvert e_{ct}\rvert >$ `slow_cte` (0.15 m) | Speed factor falls linearly to `SLOW_MIN` 0.4 at `rejoin_cte` |
| `REJOIN` | $\lvert e_{ct}\rvert >$ `rejoin_cte` (0.35 m), or heading error above 75 deg with $\lvert e_{ct}\rvert >$ `slow_cte` | Plan a way back with `plan_rejoin`; at most one plan per 2 s and `max_rejoins` (3) |
| `ABORTED` | $\lvert e_{ct}\rvert >$ `abort_cte` (1.00 m) | Stop the run |

A CBF detour (status `CBF_ACTIVE` within the last 3 s) is not drift: a rejoin waits until it ends. During `APPROACH` the speed factor stays 1.0.

**Results.** `FINISHED`, `TIME_LIMIT`, `KILLED`, `ABORTED`, `STOPPED` (Ctrl-C) and `REPLACED` (a new path arrived). If a run ends in `TRACK` with more than 50 ticks and is not `FINISHED`, the current lap is saved as a partial lap (`partial: true`).

| Function | Line | What it does |
|--------------|------|----------------------------------------|
| `RunManager` | 77 | Run logic shared by the four controllers |
| `__init__` | 88 | Declares parameters; creates `/cmd_vel`, `/run_status` (5 Hz timer), `/reference_path_viz`, `/run_kill`; `on_start` is called at each new run |
| `note_odom` | 139 | The controller calls it for each odometry message |
| `_kill_cb` | 149 | KILL and re-arm |
| `tick` | 182 | Everything up to the steering law; returns `(cte, herr, ref_pt, idx, dt)` or None |
| `shape` | 242 | Guard speed factor; clips steering; stores `steer_ctrl` and `steer_ff` |
| `command` | 257 | Publishes the Twist, feeds `LapStats`, writes one CSV row |
| `_start_run` | 294 | New run |
| `_to_lab` | 367 | Controller frame to lab frame (needed only with `rezero_odom`) |
| `_allowed` | 383 | Is a way back acceptable? Lab edge and obstacle checks |
| `_plan_way_back` | 425 | Dubins way to the start or to a point ahead |
| `_join_path` | 490 | Track the real path from an index |
| `_update_progress` | 514 | Lap counting |
| `_end_lap` | 538 | Closes a lap; logs "LAP n: RMS x cm, max y cm, at steering limit z %, t s" |
| `_update_guard` | 562 | Guard |
| `_finish` | 609 | Stops the robot, closes the log, writes `summary.json` |
| `_obstacles_flat` | 648 | Known obstacles for `summary.json` |
| `shutdown` | 658 | Ctrl-C: logs `STOPPED`, then 10 zero commands at 20 ms |
| `publish_status` | 673 | `/run_status` JSON at 5 Hz |
| `_print_status` | 692 | Console line every 2 s during a run |
| `_publish_viz` | 703 | `/reference_path_viz`, every 5th point |

The order inside `tick`: exit request, KILL or no odometry yet, start a new run, post-run zeros (`stop_ticks` = 10), odometry age, time limit, lab limit, approach deadline, tracking errors, join, laps, goal, guard. In `command`, the angular rate is $\omega = (v/L)\tan\delta$ (0 if $v < 10^{-4}$), the lap statistics skip the first 1 m of lap 1, and a `CBF_ACTIVE` status is remembered for the 3 s detour rule. In `_finish`, with `exit_on_finish` the node leaves on its next tick (not for `REPLACED`).

### 3.4 `approach_planner.py`

Plain math, no ROS. An Ackermann robot cannot turn on the spot and does not reverse here, so the shortest way between two poses is a Dubins path: arcs of a fixed radius joined by a straight (words LSL, RSR, LSR, RSL, RLR, LRL). The merge rule is in section 3.3. Constants (`:40-44`): `MERGE_AHEAD` 1.0 m, `MERGE_STEP` 0.5 m, `MERGE_MAX` 6.0 m, `MAX_TURN` $\pi$, `CHECK_DS` 0.05 m.

| Function | Line | What it does |
|-----------------|-----|--------------------------------------|
| `_mod2pi` | 52 | Wraps an angle to $[0, 2\pi)$ |
| `_dubins_words` | 66 | Up to six candidates in normalised form (radius 1, goal on +x); words without a solution are left out |
| `_sample` | 109 | Turns a word into points $[x, y, \theta, \kappa]$, one per ds; $\kappa$ is $+1/r$ (L), $-1/r$ (R), 0 (S) |
| `plan` | 144 | Shortest allowed Dubins path from pose to pose |
| `plan_rejoin` | 168 | Way back after drifting: join points 0.5 to 4 m ahead in 0.25 m steps, shortest wins, never backwards |
| `plan_to_start` | 195 | The fixed rule; returns `(table, join_idx, text)`; the caller appends `waypoints[join_idx:]` |
| `plan_to_start.no_loop` | 209 | Accepts a curve: total yaw change at most 180 deg plus 0.05 rad, and `allowed(table)` |
| `plan_to_start.result` | 216 | Builds the return value; resamples the curve at 1 cm; builds the log text |
| `_best` | 251 | Shortest allowed word on the coarser `CHECK_DS` sampling |

### 3.5 `track_tools.py`

Plain math and the `Path` message. It holds the running-track ("capsule") reference path in one place. Constants: `TRACK_RADIUS` 1.0, `TRACK_STRAIGHT` 4.0 and the centre `(TRACK_X, TRACK_Y)` = (3.25, 1.25) (`:42-45`), `TRACK_RADII` = (1.0, 0.5, 0.4) for the radius study (`:39`). The length at R1 is $2\cdot 4 + 2\pi\cdot 1 = 14.28$ m.

Options: `start_side` bottom or top (`auto` is resolved by `pick_side` before `capsule` is called), `direction` ccw or cw, `start_at` middle or begin. Point 0 is the race start: the middle of the bottom straight (heading +X when ccw) or of the top straight. With `begin` it moves back by half a straight, to where the straight starts (end of the curve). The obstacle add-on uses `begin`, so the robot has 3 m of straight before the first obstacle. With the defaults it is the same track as `matlab/lap_study/capsule_path.m`.

| Function | Line | What it does |
|--------------|------|----------------------------------------|
| `pick_side` | 59 | Resolves `auto`: bottom if the robot y is below the track centre y, else top (also top without odometry) |
| `capsule` | 76 | Track points $(x, y)$ at 1 cm; ccw from the bottom middle, cw reverses and rolls, top and begin roll the array; ValueError on an unknown option |
| `_capsule_ccw` | 104 | The ccw shape: half bottom straight, right semicircle, top straight, left semicircle, rest of the bottom straight |
| `capsule_message` | 127 | The track as `nav_msgs/Path` (frame `odom`); raises ValueError if any point is outside the lab, so a bad track fails before anything moves |

### 3.6 `track_publisher.py`

Console script `track_publisher`. It sends the capsule track instead of a hand-drawn path, once the controller is ready, then leaves. `hw_tracking.launch.py track_radius:=1.0` starts it. It requires `rezero_odom:=false`, because the track is placed in the lab frame; the launch raises otherwise.

At start it builds the track once to check it fits the lab and waits `start_delay` (5 s). It subscribes to `/run_status`. When the controller reports `WAIT_PATH`, `start_delay` has passed and at least 3 s have passed since the last send, it publishes `/reference_path`. With `start_side:=auto` it first needs the robot's y from `/hardware_odom`. When the phase is `APPROACH` or `TRACK`, it is done and `main` ends the loop.

| Function | Line | What it does |
|-------------|------|-----------------------------------------|
| `TrackPublisher` | 42 | The node; parameters `track_radius`, `track_straight`, `track_x`, `track_y`, `lab_x`, `lab_y`, `start_delay`, `start_side`, `direction`, `start_at` |
| `__init__` | 49 | Declares parameters, checks the track fits, creates publisher and subscriptions |
| `odom_cb` | 76 | Stores the robot y |
| `track` | 84 | The Path with the side chosen now |
| `status_cb` | 98 | Decides when to publish |
| `main` | 116 | `spin_once` loop until done |

### 3.7 `lab_obstacles.py`

Plain math, used only by `obstacle_lap.launch.py`. There are two cylinders of radius `OBSTACLE_RADIUS` 0.30 m (diameter 0.60 m), one on each straight, `OBSTACLE_AHEAD` 1.0 m past the middle in the driving direction. Their centres are `OBSTACLE_OUTSIDE` 0.15 m outside the track line (`:34-36`), so each one pushes the robot into the track, never towards the lab edge. The second is the first mirrored through the track centre. For ccw the centres are (4.25, 0.10) and (2.25, 2.40), for cw (2.25, 0.10) and (4.25, 2.40). The CBF keeps the marker midpoint at $r + r_{robot} + m = 0.30 + 0.20 + 0.10 = 0.60$ m from each centre, so the robot passes about 0.45 m inside the track line.

`lab_obstacles(straight, radius, cx, cy, direction, lab_x, lab_y)` (`:48`) returns the flat list `[x, y, r, x, y, r]` for the `obstacles` parameter of the bridge. The bottom obstacle is at $(c_x + s, c_y - R - 0.15)$ and the top one at $(c_x - s, c_y + R + 0.15)$ with $s = +1$ for ccw and $-1$ for cw. It raises ValueError if the straights are shorter than 2 m or an obstacle does not fit the lab.

### 3.8 `run_laps.py`

Console script `run_laps`. It is the lap-test engine. For each track radius and each controller in turn it starts the controller as a child process, hands it the track, lets it do its laps, waits `buffer_time` with the robot stopped, and starts the next one. At the end it builds the report. The flow in `LapRunner.run` (`:265`):

1. Create the session folder `<log_root>/<session_name>_<stamp>/` and write `settings.json`. `log_root` is `log_dir`, or `<workspace>/evaluation` found from the install folder, so it is the same wherever you start it.
2. Check that every radius fits the lab (`capsule_message` with the top side) before anything moves.
3. Wait for `/hardware_odom` (log every 3 s), then warn and sleep `start_delay` (5 s).
4. For each radius create the folder `R<radius>` (labels `R1`, `R0.5`). The time limit per run is

   $$t_{limit} = 60 + \frac{5\cdot \text{laps}\cdot \text{length}}{\max(0.4\,v_{nominal},\ 0.05)}\ \text{s}$$

   The limit is set at `:293`. For R1, 3 laps and 0.30 m/s that is about 1845 s. It is only a safety net: a robot that cannot get round a tight track ends as `TIME_LIMIT`, is recorded, and the test goes on.

5. For each entry of `controllers`: `name, _, mode = entry.partition(':')`, so `mpc:constraint` is controller `mpc` with `obstacle_mode` constraint for this run only. Between two entries, `buffer(buffer_time)` publishes a zero Twist every 0.2 s (KILL ends it).
6. The start side is chosen now for each controller with `pick_side(start_side, robot_y, cy)`, so a robot parked near the bottom gets the bottom straight.
7. `run_one` starts the child and waits (at most 60 s) until its `/run_status` shows its name. It re-sends the track every 3 s while the controller reports `WAIT_PATH` with no result. If `/run_status` is silent for 1.5 s, it sends a zero command and the run is `ABORTED` ("stopped reporting"). On exit it returns the controller's `result` and `reason`.
8. If a result is not `FINISHED` or `TIME_LIMIT` (`KILLED`, `ABORTED`, ...), the whole test stops, because the robot must never start again by itself after a kill or a safety stop. The message says which radii and controllers are left to run again.
9. Write `results.json`, then run `report_lap <session>` and log its exit code.

The child controller gets these parameters (`start_controller`, `:143`): `laps`, `guard`, `run_duration` (the limit above), `v_nominal`, `exit_on_finish` True, `log_dir` (the `R<r>` folder), `path_name` (the label), `rezero_odom` False, `ref_offset`, `wheelbase`, `max_steer`, `cbf_max_steer`, the approach and guard values, `guard_lab_x/y` and `obstacle_mode`, plus the remaps `/cmd_vel:=<cmd_topic>` and `/imu:=<imu_topic>`. The executable is `<package prefix>/lib/limo_mocap_suite/<name>_tracker`.

| Function | Line | What it does |
|--------------------|---------|-------------------------------|
| `LapRunner` | 70 | The node; spins in a thread, `run()` in the main thread |
| `__init__` | 77 | Declares the defaults (section 2); subscriptions `/run_kill`, `/run_status`, `/hardware_odom`; publishers `/reference_path`, `/cmd_vel` |
| `_kill_cb`, `_status_cb`, `_odom_cb` | 110, 118, 127 | Store KILL, the latest status and time, odometry time and robot y |
| `start_controller` | 143 | Starts the child with `subprocess.Popen ... --ros-args -p ...` |
| `stop_controller` | 167 | SIGINT, wait 5 s, then kill; one zero Twist |
| `run_one` | 189 | One controller on one track (steps 7 and 8) |
| `log_root` | 227 | Where test folders go |
| `buffer` | 243 | Pause with the robot held at zero |
| `run` | 265 | The whole test |
| `main` | 337 | Executor in a thread; stops and joins the thread first, because a plain `rclpy.spin` thread left running made the process abort |

### 3.9 `report_lap.py`

Console script `report_lap <folder>`. It turns run folders into `report.md`, `summary.csv` and charts, without ROS. `run_laps` calls it at the end, and you can run it again on any folder: a session folder, an `R<r>` folder, or any folder holding run folders (those with a `summary.json`). A folder with radius sub-folders (`R1`, `R0.5`, ...) gets one report per radius plus a comparison at the top level (`comparison.csv`, `comparison.png`, `report.md`).

Outputs per folder: `report.md`, `summary.csv`, `tracks.png`, `laps.png`, `error_along.png`, `steering.png`, `curvature.png`, `recovery.png`.

**Metrics per lap** are computed while driving in `stats_lap.py` and stored in `summary.json`: `rms_cte`, `max_cte`, `sat_share`, `peak_steer_use`, `peak_geometry_use`, `rms_steer_rate`, `lap_time`, `rejoins`.

**Recovery** is computed in the report from the telemetry (`recovery`, `:321`):

- A curve is a stretch with $\lvert\kappa\rvert > 0.1$ 1/m.
- `peak_cm` is the mean, over curve exits, of the largest $\lvert e_{ct}\rvert$ between the curve start and the next curve start.
- `settle_m` is the mean distance after the curve end until $\lvert e_{ct}\rvert$ stays below `SETTLED` = 0.02 m for `SETTLED_FOR` = 0.5 m of path, over the exits that settle. The distance is the length driven since the exit, summed from the logged positions $x, y$. It does not use the logged $s$, which wraps to 0 at the start line.
- `unsettled` counts exits that never settled before the next curve, and `exits` is the number measured. The last curve of a run has no following curve and is not counted.

**Charts.**

| Chart | Content |
|-------------|-----------------------------------------------|
| `tracks` | Driven lines (way to the path dotted, one colour per lap), path dashed, obstacles red |
| `laps` | RMS and maximum error per lap (bars) |
| `error_along` | Signed error against distance along the path; curves shaded where $\lvert\kappa\rvert > 0.3$ |
| `steering` | Last lap: steering sent against `steer_ff` and the limit |
| `curvature` | RMS error and 95th-percentile steering use by $\lvert\kappa\rvert$ bin; a bin needs more than 20 rows; red line at the curvature the limit allows |
| `recovery` | Bars of `peak_cm` and `settle_m` |

The bin edges are `[-0.01, 0.1, 0.4, 0.8, 1.2, 1.6, 2.0, 3.0, 6.0]`.

`report.md` has the results table (controller, result, way-to-the-path time, rejoins, RMS first lap, RMS last lap, largest error, share at the steering limit and peak steering in the last lap), the recovery table, the reason of every run that did not finish (a bold line "CTRL RESULT:" followed by the reason), a note on partial laps, and the six charts. The order in tables is plain runs first, then `constraint` runs, each as PID, LQR, LTV-MPC, NMPC. A constraint run is labelled "NAME constraints".

| Function | Line | What it does |
|--------------|------|----------------------------------------|
| `load_runs` | 65 | Loads every run: summary dict, telemetry columns as arrays, folder |
| `_on_track` | 95 | Mask of `TRACK` rows, optionally one lap |
| `_grid` | 110 | Subplot layout |
| `_label` | 121 | Legend name; adds "constraints" |
| `_shade_curves` | 132 | Grey bands where $\lvert\kappa\rvert > 0.3$ |
| `plot_tracks` | 145 | `tracks.png` |
| `plot_laps` | 181 | `laps.png` |
| `plot_error_along` | 209 | `error_along.png` |
| `plot_steering` | 243 | `steering.png` |
| `plot_curvature` | 283 | `curvature.png` |
| `recovery` | 321 | Peak and settling after curves |
| `_metres` | 366 | Table text; "never" for NaN |
| `plot_recovery` | 376 | `recovery.png` |
| `write_tables` | 399 | `summary.csv` (one row per lap) and `report.md` |
| `generate` | 447 | Whole report; recursion over `R*` folders, then `compare_radii`; SystemExit if no run folder |
| `compare_radii` | 479 | `comparison.csv`, `comparison.png`, top-level `report.md` |
| `main` | 529 | Console script; one argument, the folder |

### 3.10 `stats_lap.py`

Plain math. It keeps the per-lap statistics, the same for all four controllers. Only sums are kept, so a long run costs no memory. (Lap-to-lap learning was removed; see `docs/FUTURE_lap_learning.md`.) `SAT_AT` = 0.98 (`:21`): a tick counts as "at the limit" when $\lvert\delta_{sent}\rvert \geq 0.98\,\delta_{\max}$.

| Function | Line | What it does |
|-----------|------|-------------------------------------------|
| `LapStats` | 31 | Collects lap numbers; `record()` every tick, `end_lap()` at each lap end |
| `__init__` | 37 | Stores the steering limit |
| `_reset_lap` | 48 | Zeroes the sums |
| `record` | 69 | One tick; peaks always; squared error and maximum only if `score` |
| `end_lap` | 94 | Returns the lap dictionary and starts a new lap |

At the end of a lap, with $n_{sc}$ scored ticks and $n$ all ticks:

$$\text{rms\_cte} = \sqrt{\tfrac{1}{n_{sc}}\sum e_{ct}^2}, \quad \text{sat\_share} = \tfrac{n_{sat}}{n}, \quad \text{rms\_steer\_rate} = \sqrt{\tfrac{1}{n}\sum \dot\delta^2},$$

$$\text{peak\_steer\_use} = \frac{\max\lvert\delta_{sent}\rvert}{\delta_{\max}}, \quad \text{peak\_geometry\_use} = \frac{\max\lvert\delta_{ff}\rvert}{\delta_{\max}}.$$

The steering rate is the change since the last tick over $\max(dt, 10^{-3})$. The first 1.0 m of lap 1 (start-up transient) is not scored; this is set in `RunManager.command`.

### 3.11 `launch_prompts.py`

It asks the important settings in the terminal, so nobody has to know the arguments. The rules are in section 2.

| Function | Line | What it does |
|---------------------|-----|----------------------------------|
| `CONTROLLERS`, `START_SIDES`, `SIM` | 33-36 | Choice lists used by the lab launch files |
| `Ask` | 53 | One question: `name`, `label`, `default` (None = must be given), `choices`, `kind` (`choice`, `many`, `int`, `float`), `skip`, `strict` |
| `ask_first` | 71 | Returns an `OpaqueFunction` to put first in the `LaunchDescription` |
| `_ask_all` | 85 | For each question: command line, else prompt (if stdin is a tty), else default; prints the heading |
| `_prompt` | 119 | Asks until valid; Enter = default; Ctrl-D = default or an error |
| `_check` | 148 | Validates: numbers above 0; choices by number or value; `many` keeps the order and removes duplicates |
| `_shown` | 177 | Text for people: choice text up to the first colon |

With `strict` False a command-line value outside the choices is accepted, for example `controllers:=mpc:constraint` or `track_radius:=0.5`.

### 3.12 `sim_limo.py`

Console script `sim_limo` (`sim:=true`). It is a stand-in robot to rehearse without the lab: a bicycle model that takes `/cmd_vel` and publishes `/hardware_odom` like the bridge. The steering is deliberately imperfect so the controllers have something to correct. It is not used on the real robot.

Defaults (`:60-63`): start `x` 1.0, `y` 0.5, `yaw` 0.0; `wheelbase` 0.20; `ref_offset` 0.10; `max_steer` 0.4084; `curv_gain` 0.89; `bias` 0.02 rad; `tau` 0.12 s; `rate` 1.5 rad/s; `delay` 0.06 s; `drift_at`, `drift_steer`, `drift_for` and `freeze_at` are test hooks (0 = off). The step is 10 ms (100 Hz). Optional `obstacles` are published latched on `/hardware_obstacles`.

| Function | Line | What it does |
|----------|------|--------------------------------------------|
| `SimLimo` | 51 | The stand-in robot |
| `__init__` | 58 | Parameters, rear-axle start (odometry is `ref_offset` ahead), delay queue, publisher, subscription, 100 Hz timer |
| `cmd_cb` | 95 | Keeps speed (no reverse) and steering $\arctan(\omega L / v)$; below 1 mm/s the last steering is kept |
| `step` | 111 | Delayed steering, first-order lag, rate cap, clip; real wheel angle $\arctan(g\tan(\delta + b))$ with gain $g$ and bias $b$; integrates and publishes |
| `main` | 148 | Spin |

### 3.13 `live_plotter_node.py`

Console script `live_plotter`. It is the live off-board view of a run and holds the KILL button. It subscribes to `/reference_path`, `/hardware_odom`, `/hardware_obstacles` (latched) and `/run_status`, and publishes `/run_kill` (latched Bool) and a zero `/cmd_vel`. The KILL button needs this window and ROS to be running. It is not a hardware stop, so keep the real stop in reach.

The window shows the lab floor (path green dashed, trail orange, robot glyph with the arrow M1 to M0, obstacles red), two strip charts of the last 60 s (error in cm with the guard levels when the guard is on; steering sent against steering needed, with red limit lines), an info text and the KILL / RE-ARM button. The info text turns red when no report arrives for 1.5 s, on KILL or ABORT, and when the guard is in REJOIN or HOLD. A trail point is added only when the robot moved more than 2 cm. The history is 300 points (`deque(maxlen=300)` at 5 Hz = 60 s). The animation runs at 10 Hz.

| Function | Line | What it does |
|--------------|------|----------------------------------------|
| `LivePlotterNode` | 57 | Caches path, trail, pose, obstacles, status; KILL publisher |
| `__init__` | 67 | Parameters `rezero_odom`, `lab_x/y`, `view`; subscriptions; 10 Hz `kill_tick` |
| `path_cb` | 101 | Stores the path, clears the trail |
| `odom_cb` | 113 | Latches the frame, stores pose, appends trail point |
| `obstacles_cb` | 128 | Caches the map; the GUI redraws only on a new version |
| `status_cb` | 138 | Stores status; in `APPROACH` and `TRACK` adds `(t, cte, steer, steer_ff)` to the history |
| `set_kill` | 151 | Publishes `/run_kill`, a zero Twist, and logs |
| `kill_tick` | 159 | Repeats the zero command while KILL is on |
| `main` | 182 | Spins the node in a thread, builds the window |
| `on_kill` | 204 | Nested: KILL / RE-ARM toggle and button colour |
| `view_xy` | 237 | Nested: real X, Y to canvas |
| `update_side` | 248 | Nested: strip charts and info text |
| `update_plot` | 291 | Nested: animation callback |

### 3.14 `interactive_builder.py`

Console script `interactive_builder`. You draw a path with the mouse and it publishes it to `/reference_path` (frame `odom`). `hw_tracking.launch.py` starts it when drawing (not when `track_radius` is above 0). It subscribes to `/hardware_odom` (robot glyph) and `/hardware_obstacles`. The canvas is the lab floor, and strokes are clipped to it.

Buttons: **Smooth Path** needs at least 4 points, drops points closer than 1 mm, fits a smoothing B-spline (`scipy splprep`, $s = 0.08$), resamples at `max(300, n_raw)` points and turns the line green. **Clear** wipes the canvas. **Publish** converts canvas coordinates back to real X, Y with `from_view`. The yaw of each pose is the direction to the next point.

| Function | Line | What it does |
|---------------------|--------|-------------------------------|
| `TrajectoryPublisherNode` | 54 | Owns the `/reference_path` publisher; caches pose and obstacles |
| `__init__` | 63 | Node `interactive_builder_node`, publisher depth 10, parameter `rezero_odom`, subscriptions |
| `odom_cb` | 83 | Caches pose and latches the frame (same rule as the controllers) |
| `obstacles_cb` | 93 | Caches the map |
| `publish_trajectory` | 106 | Packs a `Path`; the last point reuses the previous direction |
| `get_dynamic_figsize` | 134 | Window size = 80 % of the screen; falls back to (12, 8) |
| `main` | 163 | Spin thread, window, buttons |
| `on_press`, `on_motion`, `on_release` | 193, 207, 219 | Nested: mouse stroke; a press starts a new stroke |
| `refresh_overlay` | 233 | Nested: 5 Hz robot glyph, obstacles, status text |
| `apply_smoothing` | 270 | Nested: Smooth Path |
| `clear_canvas` | 293 | Nested: Clear |
| `publish_data` | 305 | Nested: Publish |

### 3.15 `plot_helpers.py`

Shared Matplotlib helpers for the builder, the live plotter and `limo_test_suite`: the lab canvas with projector styling (thick red axes through (0, 0), 1 m grid, large labels), the view transform, the start-frame transform, the robot glyph and obstacle circles. Constants: `BODY_LENGTH` 0.32, `BODY_WIDTH` 0.22, `MARKER_OFFSET` 0.12 m (drawing only), `WALL_RADIUS` 100 m (an obstacle this big is drawn as a dashed wall).

| Function | Line | What it does |
|-------------------------|---------|--------------------------|
| `quat_to_yaw` | 56 | Planar yaw from a quaternion |
| `init_gui_ros` | 69 | `init_ros` plus Ctrl-C closes all windows, so node cleanup runs also under `ros2 launch` |
| `start_spin_thread` | 81 | Spins the node in a daemon thread |
| `declare_lab_params` | 100 | `lab_x`, `lab_y` parameters |
| `declare_view_param` | 120 | `view`: `behind` (+X up, +Y left) or `topdown` |
| `style_lab_axes` | 141 | Limits, equal aspect, 1 m major and 0.5 m minor grid, red axes |
| `lab_legend` | 184 | Legend clear of labels |
| `to_view` | 198 | Lab to plot coordinates; behind: $u = -y,\ v = x,\ \text{yaw} + \pi/2$ |
| `from_view` | 210 | Inverse; behind: $x = v,\ y = -u$ |
| `StartFrame` | 225 | Mirrors `rezero_odom` so the GUIs draw in the controller frame |
| `StartFrame.__init__`, `latch`, `ready`, `apply` | 232, 241, 251, 260 | False = identity; first call wins; origin known; $p' = R(-\text{yaw}_0)(p - p_0)$ |
| `RobotGlyph` | 278 | Body rectangle, arrow M1 to M0, red dot front (id 0), blue dot rear (id 1) |
| `RobotGlyph.__init__`, `update`, `set_visible`, `artists` | 285, 305, 322, 331 | Create, move, show or hide, list the parts |
| `draw_obstacles` | 345 | Removes old patches, draws the new map; $r < 100$ filled circle, otherwise dashed wall |

### 3.16 `limo_test_suite.py`

Console script `limo_test_suite`, launch `limo_test.launch.py`. It is the open-loop actuator test. It plays a test path as `/cmd_vel` at 50 Hz and shows, seen from behind the robot, where it should go and how the front wheels should be steered. There is no odometry, no mocap and no correction. Use it with the wheels lifted, or on the floor with clear space. It publishes only on `/<robot_ns>/cmd_vel`.

Paths: `straight` (2.0 m), `circle` (one full left circle, radius 0.80 m), `figure8` (left circle, then right circle) and `drawn` (the last `/reference_path`, in its own start frame). The command is $s = v\,t$, $\delta = \mathrm{clip}(\arctan(L\kappa(s)), \pm\delta_{\max})$, $\omega = (v/L)\tan\delta$. The phases are `IDLE`, `COUNTDOWN` (zeros for `start_delay` 3 s), `RUNNING`, `DONE`. In `IDLE` and `DONE` nothing is published, and STOP sends one zero Twist. Defaults: speed 0.20 (launch), `wheelbase` 0.20, `max_steer` 0.4084, `repeat` False, `autostart` False. The path sampling step is `DS` = 0.01 m (`:60`).

| Function | Line | What it does |
|---------------------|----------|-----------------------------|
| `path_from_segments` | 71 | Path from `(length, kappa)` pieces, integrated every DS |
| `path_from_points` | 94 | Drawn points to the same format: drops points under 1 cm apart (needs at least 4), rotates into the start frame, resamples, finite-difference curvature smoothed with a 9 cm moving average |
| `LimoTestSuiteNode` | 130 | Holds the path, plays it open loop, dead-reckons the expected pose from the commands |
| `__init__` | 137 | Parameters (also `radius`, `straight_length`, `auto_place`, `start_x/y`, `lab_x/y`, `view`, `robot_ns`), built-in paths, publisher, `/reference_path` subscription, 50 Hz timer |
| `path_cb` | 197 | Stores the drawn path; ignored if under 4 distinct points |
| `reset_expected` | 216 | Expected pose back to the start |
| `select`, `start`, `stop`, `now` | 229, 241, 250, 257 | Choose path (not while running), begin countdown, abort with a zero Twist, node clock |
| `placement` | 268 | Where the path start is drawn (display only) |
| `duration` | 280 | Path length over speed |
| `tick` | 293 | 50 Hz: countdown zeros, running command and expected pose, end handling |
| `snapshot` | 341 | Thread-safe copy for the GUI |
| `WheelPanel` | 359 | Front wheels rotated by the commanded steering, green speed arrow |
| `WheelPanel.__init__`, `_rect`, `update` | 367, 398, 410 | Create, wheel corners, rotate |
| `main` | 439 | Window, radio buttons, Start and STOP; always a zero Twist on exit |
| `lab`, `update_plot` | 472, 500 | Nested: path frame to canvas; 10 Hz redraw |

### 3.17 `robot_topics.py` and `ros_compat.py`

| Function | Line | What it does |
|---------------------|-----|----------------------------------|
| `ROBOT_TOPICS`, `DEFAULT_ROBOT_NS` | 25, 26 | Robot topics `('/cmd_vel', '/imu')`; namespace `limo1` |
| `robot_remappings` | 38 | List of `(from, to)` for the launch files. `limo1` gives `/cmd_vel` to `/limo1/cmd_vel` and `/imu` to `/limo1/imu`; `none`, `/` or empty gives no remap |
| `init_ros` (`ros_compat.py`) | 42 | `rclpy.init` that keeps the ROS context alive on Ctrl-C |

Use `robot_ns:=none` on the command line, because `ros2 launch` rejects an empty value. On Humble `init_ros` passes `signal_handler_options=SignalHandlerOptions.NO`, so the final zero Twist can still be published. On Foxy it calls plain `rclpy.init`, whose handler already behaves this way. The same code therefore runs on Foxy (Python 3.8) and Humble (Python 3.10).

### 3.18 Launch files in detail

| File and function | Line | What it does |
|---------------------------|----|-----------------------------|
| `hw_check.launch.py` `_nodes` | 41 | Bridge (if `bridge:=true`, first) and `hw_check`; `Shutdown` when `hw_check` exits |
| `hw_check.launch.py` `generate_launch_description` | 62 | Arguments `drive`, `speed` 0.15, `ref_offset` 0.10, `max_range` 3.0, `robot_ns`, `bridge` true, `log_dir` evaluation |
| `limo_test.launch.py` `_test_node` | 45 | `limo_test_suite` with `path`, `speed`, `view`, `robot_ns` |
| `limo_test.launch.py` `generate_launch_description` | 63 | Arguments `robot_ns`, `path`, `speed`, `view` (behind) |
| `hw_tracking.launch.py` `_controller_node` | 70 | The chosen `<ctrl>_tracker` with its parameters; raises on an unknown controller |
| `hw_tracking.launch.py` `_bridge_node` | 108 | Bridge with `obstacles` and `obstacle_markers` only if non-empty |
| `hw_tracking.launch.py` `_track_node` | 132 | `track_publisher` if `track_radius` is above 0; raises if `rezero_odom` is true |
| `hw_tracking.launch.py` `_gui_nodes` | 159 | `interactive_builder` (only when drawing) and `live_plotter` with the remap |
| `hw_tracking.launch.py` `generate_launch_description` | 189 | Declares all arguments, asks `controller` and `track_radius` |
| `record_lap.launch.py` `_include` | 49 | Validates the controller name, includes `run_laps.launch.py` |
| `record_lap.launch.py` `generate_launch_description` | 69 | Arguments `controller`, `laps`, `start_side`, `sim` |
| `compare_lap.launch.py` `generate_launch_description` | 48 | Arguments `controllers`, `laps`, `buffer_time`, `start_side`, `sim` |
| `obstacle_lap.launch.py` `_include` | 62 | Computes the obstacles with `lab_obstacles`, logs them, includes `run_laps.launch.py` |
| `obstacle_lap.launch.py` `generate_launch_description` | 97 | Arguments `compare_modes`, `controllers`, `laps`, `start_side`, `sim`, `direction` |
| `r05_lap.launch.py` `generate_launch_description` | 49 | Like compare, with `track_radii:=0.5` |
| `run_laps.launch.py` `_nodes` | 58 | `sim_limo` or `phasespace_bridge`; `live_plotter`; `run_laps` with all test parameters |
| `run_laps.launch.py` `generate_launch_description` | 105 | The argument table of section 2 |

`hw_tracking.launch.py` has the defaults `laps` 1 (0 = until stopped), `log_dir` evaluation, `approach` true, `robot_radius` 0.20, `safety_margin` 0.10, `cbf_alpha` 0.6, `cbf_max_steer` 0.0 (use `max_steer`) and `robot_ns` limo1. It does not pass `v_nominal`, so the controller's own default (0.40 m/s) applies. In `run_laps.launch.py` the bridge is not remapped, because its topics are not the robot's.

### 3.19 `tools/send_logs.sh`

It copies the log folders from the lab machine to the development machine over `scp` with one password prompt. It needs only coreutils and openssh-client, and never uses `sudo` interactively.

1. Defaults at the top: `REMOTE_USER=alienator`, `REMOTE_DEST=/media/alienator/Storage/Jay/Robotics/ALNC/HW_Ackermann_Drive/evaluation/from_lab/`. Options: `-d DIR` (log folder, default `<repo>/evaluation`), `-i IP|HOST`, `-u USER`, `-p PATH`, `--dry-run`, `--check-receiver`, `-h`.
2. If `ssh` or `scp` is missing, it installs `openssh-client` only with passwordless sudo, otherwise it prints the command and exits.
3. It lists the folders of `evaluation/` newest first with size and a `[sent]` mark (from `.sent_log`); `from_lab` is skipped. Selection: Enter = all not yet sent, `a` = all, or numbers like `1 3 4`.
4. It asks for the destination IP or hostname (validated) and checks the user name.
5. It opens one master ssh connection (`ControlMaster`, `ControlPersist=120`), runs `mkdir -p` on the receiver, then `scp -r` for each folder over the same connection.
6. It verifies each folder with `du -sh` on the receiver. Only verified folders are appended to `.sent_log`. The summary lists Sent OK, Unverified and Failed; the exit code is 1 if any failed or is unverified.

`--check-receiver`, run on the receiving PC, checks `openssh-server`, `sshd`, port 22, `ufw`, `PasswordAuthentication` and the destination folder. It prints the IP addresses and the command to type on the lab machine, and prints the `sudo` commands to fix problems without running them.

### 3.20 `tools/hw_test/` and `tools/planner_check.py`

`tools/hw_test/` rehearses the hardware stack without robot and mocap. It is not part of the ROS build (`COLCON_IGNORE`).

| File | What it does |
|------------------|------------------------------------------|
| `run_hw_tests.py` | Runs the real `phasespace_bridge` and `hw_check` or a controller against a stand-in robot and fake markers, in a separate build under `~/.cache/limo_hw_test`, with `ROS_DOMAIN_ID` 87 and `ROS_LOCALHOST_ONLY` 1 |
| `fake_mocap.py` | Turns the truth of the stand-in robot (`/sim_truth`) into `/phasespace/markers` with chosen faults |
| `src/phasespace_msgs/` | Stub message package (`Marker.msg`, `Markers.msg`); never source it where the real package exists |

The cases of `run_hw_tests.py` are `normal` (expects "OK to run tracking" with warnings, because `sim_limo` is imperfect), `mirror`, `swap` and `nolisten` (each expects a FAIL naming the fault) and `glitch` (dropped frames are reported, and a `pid` run on a short capsule finishes). Run it with `python3 tools/hw_test/run_hw_tests.py [case ...]`; all cases take about 4 minutes. The fake markers sit 0.1 m ahead of and behind the pose (0.2 m apart) with 0.5 mm Gaussian noise at 100 Hz. `mirror` negates y, `swap` exchanges the ids, and `glitch` makes the front marker jump 0.6 m every 200th frame, swaps the markers for one frame every 330th, and occludes for 0.4 s every 1500 frames.

`tools/planner_check.py` checks the way-to-the-start rule without ROS. It draws the plan from six start poses and sweeps a grid of start poses (every 0.5 m, 8 headings) on the R1 track with the edge rules of `run_manager`. It writes `docs/figures/fixed_merge_rule.png` and prints the table of outcomes.

## 4. Logs

### 4.1 Folder layout

The lap files (through `run_laps`) write:

```
<workspace>/evaluation/                  (log_dir empty; send_logs.sh sends it)
  <session>_<YYYYmmdd_HHMMSS>/           session: record_<ctrl> | compare |
  |                                      obstacle_lap | obstacle_modes |
  |                                      r05_lap | run_laps
    settings.json      all run_laps parameters
    results.json       {"R1_pid": ["FINISHED", "3 laps done"], ...}
    report.md  summary.csv  comparison.*  (comparison only for several radii)
    R1/  (or R0.5/ ...)  one folder per radius
      <stamp>_<controller>_R1/    one folder per controller run
        telemetry.csv     one row per control tick
        summary.json      result and per-lap statistics
      report.md  summary.csv  tracks.png  laps.png  error_along.png
      steering.png  curvature.png  recovery.png
```

`hw_tracking.launch.py` runs write `<log_dir>/<stamp>_<controller>_<path_name>/` directly (`log_dir` default `evaluation`, relative to the folder you started in; `path_name` default `drawn`). `hw_check` writes `<log_dir>/hw_check_<stamp>/report.json`. `send_logs.sh` marks sent folders in `evaluation/.sent_log`, and received folders go to `evaluation/from_lab/` on the PC. A `constraint` run has the same controller name in its folder (`..._mpc_R1`); the mode is in `summary.json` (`obstacle_mode`).

### 4.2 `telemetry.csv`

It has 23 columns (header written in `_start_run`, `run_manager.py:334`; row in `command`, `:257`). There is one row per call of `RunManager.command`, so one per control tick while a run is open (`APPROACH` and `TRACK`).

| Column | Unit | Meaning |
|------------------|-----|------------------------------------|
| `t` | s | Time since the run started (includes the approach) |
| `phase` | text | `APPROACH` or `TRACK` |
| `lap` | 1.. | Current lap number |
| `x`, `y` | m | Pose of the tracked point, controller frame |
| `yaw` | rad | Heading |
| `v_actual` | m/s | Speed from the `/hardware_odom` twist (bridge low-pass) |
| `v_cmd` | m/s | Speed commanded (after guard factor and CBF) |
| `omega_cmd` | rad/s | Yaw rate in the Twist |
| `steer_rad` | rad | Steering commanded (clipped to `max_steer`) |
| `steer_ff` | rad | Steering the path needs: $\arctan(L\kappa)$ at the nearest point |
| `steer_ctrl` | rad | Steering the controller asked for, before guard and clip |
| `x_ref`, `y_ref`, `yaw_ref` | m, rad | Nearest path point |
| `kappa` | 1/m | Path curvature there (left positive) |
| `s` | m | Table index times spacing; during `APPROACH` that is the way table |
| `cte` | m | Signed cross-track error |
| `heading_err` | rad | Heading error |
| `min_clearance` | m | Smallest clearance to a known obstacle (CBF layer) |
| `safety_status` | text | CBF or MPC status, for example `CBF_ACTIVE` |
| `guard` | text | `OFF`, `OK`, `SLOW`, `REJOIN`, `HOLD` (odometry lost) |
| `solve_ms` | ms | Controller solve time (0.0 for PID and LQR) |

### 4.3 `summary.json`

Written by `_finish` (`run_manager.py:609`).

| Field | Meaning |
|------------------------|------------------------------------|
| `controller`, `path_name` | `pid`, `lqr`, `mpc` or `nmpc`; label (`R1` ...) |
| `obstacle_mode` | `cbf` or `constraint` |
| `result`, `reason` | One of the results of section 3.3 and the text |
| `guard` | Guard on or off |
| `laps_requested`, `closed_path` | Laps asked (1 for an open path); bool |
| `path_length_m`, `path_peak_curvature` | Main path length; maximum $\lvert\kappa\rvert$ |
| `curvature_limit`, `max_steer` | $\tan(\delta_{\max})/L$ (about 2.16 1/m); steering limit |
| `approach_time_s` | From run start until the robot joined the path (0 if it started on it) |
| `rejoins`, `duration_s` | Guard rejoins; run time |
| `laps` | One dictionary per lap: the eight metrics of section 3.9, `lap`, `lap_time`, and `partial: true` for a lap cut short |
| `obstacles` | Flat `[x, y, r, ...]` of the known obstacles |

`/run_status` (JSON, 5 Hz) has: `controller`, `phase` (`KILLED` if killed), `lap`, `laps`, `cte`, `herr`, `steer`, `steer_ff`, `max_steer`, `guard`, `guard_on`, `slow_cte`, `rejoin_cte`, `rms_last`, `result`, `reason`, `run_dir`.

The `hw_check` `report.json` has `settings`, `findings` (list of [level, name, text]), `measured` (per segment: speed, kappa, beta, dyaw; `coast`; `start_lag`) and `suggestions` (`ref_offset`, `tightest_radius_m`, `real_steer_limit_rad`).

## 5. Likely questions

**Which frame is the pose in, and which way is the heading?**
ROS right-handed: +x forward, +y left, yaw counter-clockwise. The pose is the midpoint of the two markers. The heading is the vector from the rear marker (id 1) to the front marker (id 0). The bridge does not mirror or rotate anything.

**What if a marker jumps or is swapped for a frame?**
`PoseGate` drops the frame if the marker distance differs from the learned one by more than 3 cm, or if the pose moved more than 5 cm plus 1.5 m/s times $dt$ (or turned more than 0.25 rad plus 3 rad/s times $dt$). The 10th rejected jump in a row is accepted, because the robot was really moved. Dropped frames are counted and reported every 5 s.

**What happens when odometry stops?**
The bridge publishes nothing. In `RunManager.tick`, odometry older than `odom_timeout` (0.5 s) gives a zero Twist at every tick and no control. It resumes when odometry returns. With `guard:=true` it aborts after `odom_abort` (3.0 s). A new run never starts from a stale pose.

**How does KILL work, and what are its limits?**
The live plotter button publishes a latched `/run_kill` and zero Twists (10 Hz). Every controller stops, finishes the run as `KILLED` and refuses to move until RE-ARM; after RE-ARM it waits for a new path. `run_laps` stops the whole test. It depends on the plotter window and on ROS (WiFi). It is not a hardware stop, so the real stop must stay in reach.

**How does the robot get to the start of the track?**
`plan_to_start` takes the closest path point in front of the robot that runs within 90 deg of its heading, merges 1 m further on with the shortest Dubins curve (radius 0.60 m), and then follows the real path to the start. The curve may not loop (more than 180 deg), leave the allowed area, or come within 0.5 m of an obstacle edge; otherwise the merge point moves 0.5 m further, up to 6 m. The last resort is the shortest curve straight to the start.

**What if the robot stands near the lab edge, facing out?**
`_plan_way_back` tries 0.25 m inside the edges, then the lab, then up to 0.3 m past the edge (with a warning; not with the guard). If none works, the run ends `ABORTED` and the robot is not moved.

**How are laps counted, and what is not scored?**
By forward progress along the path indices, not by crossing a line. A lap is complete when the progress reaches $\text{lap}\cdot n$. The first 1.0 m of lap 1 is left out of the RMS and the maximum error. All ticks count for the steering-limit share.

**What is "at the steering limit"?**
A tick with $\lvert\delta_{sent}\rvert \geq 0.98\,\delta_{\max}$. `sat_share` is those ticks divided by all ticks of the lap. `peak_steer_use` is the peak steering over the limit, and `peak_geometry_use` is what the path itself needs over the limit.

**Which side does the robot start on?**
`start_side:=auto` chooses the top straight, unless the robot is below the track centre line ($y < 1.25$ m), then the bottom. In `run_laps` this is decided for each controller separately, when it starts. The direction defaults to ccw, and `start_at` to `middle` (`begin` in the obstacle add-on).

**What decides the order and the pause in the compare test?**
The order of `controllers` and `buffer_time` (10 s of zero commands). The next controller then drives itself to the start with the same rule, logged as `APPROACH` rows, not as laps. A kill or safety stop ends the whole test. A `TIME_LIMIT` is recorded and the test goes on.

**What is the time limit of a run?**
$60 + 5\cdot\text{laps}\cdot\text{length}/\max(0.4\,v_{nominal}, 0.05)$ seconds, about 1845 s for R1, 3 laps and 0.30 m/s. It is only a safety net.

**Why does `run_laps` use 0.30 m/s and `hw_tracking` 0.40 m/s?**
`run_laps` passes `v_nominal` 0.30 to every controller. `hw_tracking` does not pass it, so the controllers' own default applies.

**How are RMS and recovery computed?**
RMS is $\sqrt{\text{mean}(e_{ct}^2)}$ over the scored ticks of a lap, computed live by `LapStats`. Recovery is computed in the report, per curve exit: the peak $\lvert e_{ct}\rvert$ from the curve start to the next curve start, and the driven distance after the curve end until $\lvert e_{ct}\rvert$ stays below 2 cm for 0.5 m.

**What does `hw_check` tell me before a test?**
Standing still: mocap rate (at least 50 Hz), gaps (at most 0.2 s), jitter (at most 5 mm and 1 deg), and whether `/cmd_vel` has a subscriber. Driving: speed gain, steering trim, left and right curvature gain, the `ref_offset` estimate, the real tightest radius, a mirrored frame, swapped markers and whether the robot stops by itself when commands stop.

**What is `sim_limo` for, and is it a good model?**
It is a rehearsal without the lab (`sim:=true`): a bicycle model with deliberate faults (curvature gain 0.89, trim 0.02 rad, lag 0.12 s, rate limit 1.5 rad/s, delay 0.06 s). It checks the software chain (planner, run manager, logging, report). It does not validate the controllers on the real robot.

**Why is the open-loop `limo_test` useful?**
It sends commands from the path geometry only, so any difference between the expected and the real wheel angle or motion comes from the robot or the connection, not from a controller. It checks the namespace, the steering direction and the actuator range.

**How do I get the logs to my PC?**
Run `tools/send_logs.sh` on the lab machine. On the PC, run `tools/send_logs.sh --check-receiver` first. The logs arrive in `evaluation/from_lab/`.

**How was the hardware chain tested without the lab?**
With `tools/hw_test/run_hw_tests.py`: `sim_limo`, fake markers (with mirror, swap and glitch faults), the real bridge, then the real `hw_check` or `pid_tracker`.

**How does the package run on both Foxy and Humble?**
`ros_compat.init_ros`: on Humble the default SIGINT handler would shut ROS down before the final zero Twist is sent, so it is disabled; on Foxy `rclpy.init` already behaves that way.

**Why does the same code serve a drawn path and the capsule track?**
The controller only sees `/reference_path`. `track_publisher` and `run_laps` publish the same message type as the builder, and `RunManager` handles approach, laps and logging in the same way for every path.
