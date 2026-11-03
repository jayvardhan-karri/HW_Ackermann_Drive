# MATLAB LIMO Controller Suite

MATLAB version of the LIMO trajectory-tracking stack. You can draw the reference path free-hand on the lab grid and smooth it. The four controllers are then simulated (PID, LQR, LTV-MPC, NMPC), compared, animated, and can be run on the real LIMO through ROS 2. The controllers use the same models and tuning as the Python package (`src/limo_mocap_suite`), so results are comparable.

Everything is plain `.m` files. `main_track.m` is split into `%%` sections, so you can run it section by section (Ctrl+Enter) or open it as a Live Script (right-click › *Open as Live Script*).

---

## 1. Requirements

Tested with **MATLAB R2026a** on Ubuntu 22.04.

| Product | Needed for | Required? |
|---|---|---|
| **MATLAB** (base) | Everything: drawing, smoothing (`smoothdata`), reference, PID, LTV-MPC, NMPC (own solver), simulation, plots | **Yes** |
| **Control System Toolbox** | `lqr` in the LQR controller | No. If it is missing, a built-in Riccati solver (Hamiltonian / ordered Schur, base MATLAB) is used; same result. |
| **ROS Toolbox** | `run_on_limo.m`: running a controller on the real robot (`ros2node`, `ros2subscriber`, `ros2publisher`) | Only for hardware runs |
| Optimization Toolbox | — | **Not needed**: the NMPC uses its own projected Levenberg–Marquardt solver (Section 4.4) |
| Curve Fitting Toolbox | — | **Not needed**: smoothing uses `smoothdata` instead of `csaps`/B-splines |
| Image Processing Toolbox | — | **Not needed**: free-hand drawing uses figure mouse callbacks, not `drawfreehand` |

For hardware runs you also need the **Python `phasespace_bridge`** running (it turns PhaseSpace markers into `/hardware_odom` and `/hardware_scan`), on the same `ROS_DOMAIN_ID` as `P.ros.domain_id`.

---

## 2. Quick start

```matlab
cd matlab
main_track                 % draw a path, simulate all four controllers, plot, animate
```

Other path sources (set the variable before running the script):

```matlab
PATH_MODE = 'preset'; PRESET = 'figure8'; main_track     % figure8 | circle | oval | scurve | straight
PATH_MODE = 'file';   PATH_FILE = 'results/limo_path.mat'; main_track   % reuse a saved path
CONTROLLERS = {'pid', 'nmpc'}; main_track                  % only some controllers
```

On the real robot (wheels lifted for the first try):

```matlab
startup_limo; P = config_limo; P.ros.domain_id = 0;      % your ROS_DOMAIN_ID
[x, y] = draw_path(P);                                     % or preset_path('scurve')
ref = build_reference(x, y, P);
result = run_on_limo(ref, 'nmpc', P);                      % close the figure / Ctrl-C to stop
```

Outputs go to `matlab/results/`: `metrics.csv`, `comparison_paths.png`, `comparison_signals.png`, `logs.mat`, and `limo_path.mat` (the last path used).

---

## 3. Folder layout

```
matlab/
├── README.md               this file
├── main_track.m            entry script (sections 1-7)
├── config_limo.m           every parameter in one struct P
├── startup_limo.m          adds the sub-folders to the MATLAB path
├── path_tools/
│   ├── draw_path.m         free-hand path builder window (mouse drawing, smoothing, presets)
│   ├── smooth_path.m       multi-pass arc-length Gaussian smoothing
│   ├── preset_path.m       built-in paths in the lab frame
│   ├── build_reference.m   waypoint table: heading, curvature, speed profile, closed/open
│   ├── ref_index.m         index wrap (closed) / clamp (open)
│   └── tracking_errors.m   nearest point, look-ahead, cross-track and heading error, goal
├── controllers/
│   ├── ctrl_pid.m          PID + curvature feed-forward
│   ├── ctrl_lqr.m          gain-scheduled LQR (with gain cache)
│   ├── solve_lqr_gain.m    LQR gain (lqr() or built-in Riccati solver)
│   ├── ctrl_ltvmpc.m       LTV-MPC by backward Riccati recursion
│   ├── ctrl_nmpc.m         nonlinear MPC + projected Levenberg-Marquardt solver
│   ├── sample_horizon.m    reference points over the prediction horizon
│   └── speed_law.m         shared speed command
├── safety/
│   ├── synthetic_scan.m    ray-cast 240° LiDAR from the obstacle list
│   └── lidar_safety_filter.m  speed limiter + repulsive steering
├── sim/
│   ├── controller_registry.m  name -> step function, rate, look-ahead
│   ├── run_simulation.m    closed-loop simulation (bicycle model + servo lag)
│   ├── bicycle_step.m      kinematic bicycle integration (RK2)
│   ├── compute_metrics.m   RMS / max errors, steering rate, solve time
│   ├── plot_comparison.m   lab-grid trajectories + time plots, PNG export
│   └── animate_run.m       playback with the robot glyph
├── ros2/
│   ├── run_on_limo.m       run a controller on the robot through ROS 2
│   └── publish_path_ros2.m send the path as nav_msgs/Path
├── utils/
│   ├── style_lab_axes.m    projector grid: red axes, 1 m grid, large labels
│   ├── robot_glyph.m       LIMO glyph with an M1 -> M0 heading arrow
│   ├── light_figure.m      figure forced to the light theme
│   └── wrap_angle.m
└── results/                generated outputs
```

---

## 4. How the code works

### 4.1 Data flow

```
draw_path / preset_path / .mat  ->  smooth_path  ->  build_reference  ->  ref (waypoint table)
                                                                            |
               +------------------------------------------------------------+
               v
  every control tick:  tracking_errors(ref, pose)  ->  err (cte, herr, look-ahead row, nearest)
                       controller step(err, pose, ref, mem)  ->  delta, v
                       lidar_safety_filter(scan)  ->  delta, v (slowed / pushed away)
                       clip delta,  omega = v tan(delta) / L
               |                                        |
     run_simulation: bicycle_step (plant)      run_on_limo: publish /cmd_vel, read /hardware_odom
```

Every controller has the same signature, `[delta, v, mem] = step(err, pose, ref, mem, P, dt)`. `mem` is a struct the controller keeps between calls (integrator, previous error, warm start, gain cache). The simulation and the ROS runner can therefore swap controllers freely; `controller_registry.m` maps the name to the function, its rate (50 Hz, NMPC 25 Hz) and its look-ahead.

### 4.2 Reference path

**Drawing (`draw_path.m`).** A figure on the lab grid with the obstacles drawn. Pressing on the axes starts a new stroke, dragging appends points (clipped to the lab), and releasing stops. It uses `WindowButtonDownFcn` / `WindowButtonMotionFcn` / `WindowButtonUpFcn` callbacks, so no toolbox is needed. The UI elements are:

| Element | Purpose |
|---|---|
| Blue line | the stroke as drawn |
| Green thick line | after **Smooth** |
| Green dot / red cross | start / end |
| Slider *Smoothing window (m)* | 0.05–1.00 m; re-smooths immediately if already smoothed |
| **Smooth** | runs `smooth_path` |
| **Clear** | erase |
| *Load preset...* menu | figure8, circle, oval, scurve, straight |
| **Save .mat** | saves `x`, `y` (reload with `PATH_MODE='file'`) |
| **Use this path** (or closing the window) | returns the path (smoothed if smoothed) |
| Status line | points, length, closed loop / open path |

A stroke whose ends are closer than `P.ref.closed_tol` (0.15 m) is a **closed loop**, driven repeatedly. Anything else is an **open path**, and the robot stops at its end.

**Smoothing (`smooth_path.m`).** Mouse strokes zig-zag, and curvature, which depends on second derivatives, would explode. The function:
1. drops repeated points (< 1 mm apart);
2. runs three passes of: re-sample every 1 cm of arc length, then a Gaussian moving average (`smoothdata`) over `window` metres. Re-sampling between passes keeps the window a true distance even when jitter inflates the raw arc length. Closed strokes are padded circularly so the seam is smooth too.

**Waypoint table (`build_reference.m`).** This is the port of the Python `TrajectorySubscriber.path_cb`:
1. Re-sample every `P.ref.ds` = 1 cm.
2. Heading `yaw = atan2(dy, dx)` and signed curvature `κ = (x'y'' − y'x'') / (x'² + y'²)^1.5` from central differences (circular for closed loops). `κ` is then lightly smoothed over 10 cm.
3. Speed profile `v_ref = v_nominal / (1 + 0.75 |κ|)`.
4. `ref.W = [x y yaw v κ]`, plus `n`, `closed` and `length`.

The tightest curve the LIMO can follow is `κ_max = tan(28°) / 0.2 = 2.66 1/m` (radius 0.38 m). `main_track` prints the path's maximum `|κ|` next to this limit.

**Tracking errors (`tracking_errors.m`).** This is the port of `compute_tracking_errors`:
1. Nearest waypoint inside a sliding window `[last − 25, last + 160]`. This is fast, and it cannot jump to the other lobe where a figure-8 crosses itself. If the nearest point is more than 1.5 m away, it falls back to a global search.
2. The look-ahead point is `nearest + lookahead` samples ahead. `ref_index` wraps the index on a closed loop and clamps it on an open path.
3. `cte = −sin(yaw_r)(x_r − x) + cos(yaw_r)(y_r − y)`, which is positive when the path lies to the robot's left. `herr = wrap(yaw_lookahead − yaw)`.
4. Open path: `goal` is set when the nearest point is the last one and the robot is within 5 cm of it, or past it.

### 4.3 Robot model and shared laws

Kinematic bicycle (`bicycle_step.m`, midpoint/RK2 integration):

```
x' = v cos(yaw),   y' = v sin(yaw),   yaw' = v/L tan(delta),   L = 0.20 m, |delta| <= 0.4887 rad
```

* **Feed-forward:** `delta_ff = atan(L κ)` holds a curve of curvature κ with zero error.
* **Speed law** (`speed_law.m`, all controllers): `v = v_ref · max(0.4, 1 − 0.5|delta|/delta_max)`, which is half speed at full lock.
* **Twist:** `omega = v tan(delta) / L` (used in `run_on_limo`; the LIMO base converts back to steering).

### 4.4 Controllers

**PID (`ctrl_pid.m`)**
`delta = atan(Lκ) + kp·cte + ki·∫cte + kd·d(cte)/dt + kh·herr`, with the integral clamped to ±0.5 (anti-windup). The derivative starts at the first error, so there is no kick. Defaults are kp 1.8, ki 0.05, kd 0.45, kh 1.15, look-ahead 10.

**LQR (`ctrl_lqr.m`, `solve_lqr_gain.m`)**
Linear error model around the feed-forward steering:
```
d/dt [cte; herr] = [0 v; 0 0] [cte; herr] + [0; −v(1+(Lκ)²)/L] (delta − delta_ff)
```
Q = diag(450, 35), R = 0.5. The gain is `K = lqr(A, B, Q, R)` and the law is `delta = delta_ff − K [cte; herr]`. A and B change with the look-ahead speed and curvature, so the gain is re-computed: it is gain-scheduled, and memoised on (v rounded to 1 mm/s, κ rounded to 0.01 1/m). Without the Control System Toolbox, `care_schur` solves the Riccati equation from the stable invariant subspace of the Hamiltonian `[A −BR⁻¹Bᵀ; −Q −Aᵀ]` (ordered real Schur form). This MATLAB code uses the physical (negative) `B` with `u = −Kx`. The Python node uses `+B` with `+K`, which is the same closed loop (Theory_Explanation, Section 5.2). Check: `lqr([0 0.4;0 0],[0;-2],diag([40 8]),0.8)` returns `[-7.071 -3.582]`, as in the worked example.

**LTV-MPC (`ctrl_ltvmpc.m`)**
This is a port of the Python LTV-MPC with identical matrices and weights.
1. `sample_horizon` picks N = 16 reference rows, 0.04 s apart, advancing `v·dt/ds` waypoints per step.
2. Augmented state `z = [cte; d(cte)/dt; herr; previous steering deviation]`. For each step `k`, `A_k` and `B_k` are built from `(v_k, κ_k)`.
3. Backward Riccati recursion from `P_N = 3Q`: `K_k = (R + BᵀPB)⁻¹BᵀPA`, then `P = Q + AᵀP(A − BK_k)`. This is the closed-form solution of the unconstrained finite-horizon QP.
4. `delta = Σ w_k atan(Lκ_k) − K₁ z + 0.06 ∫cte`, where the preview weights are `w = [0.45 0.25 0.15 0.10 0.05]`. The result is clipped.

**NMPC (`ctrl_nmpc.m`)**
Decision variables are the N = 10 steering angles, 0.08 s apart (0.8 s of preview). For a candidate `u`, the robot is forward-simulated with the **nonlinear** model and compared with reference sample k:
```
J(u) = Σ_k  q_cte·cte_k² + q_heading·herr_k² + rd·(u_k − u_{k−1})²,    |u_k| ≤ delta_max
```
with q_cte 40, q_heading 8 and rd 2.5. The Python node minimises this with SciPy's SLSQP. Without the Optimization Toolbox, this code uses the fact that J is a **sum of squares**, `J = ‖r(u)‖²`, and minimises it with a **projected Levenberg–Marquardt** method (`nmpc_solve`):
1. Jacobian of `r` by forward differences (N + 1 roll-outs of 10 steps each, which is cheap).
2. Step `du = −(JᵀJ + λI)⁻¹ Jᵀr`; project `u + du` onto the bounds.
3. Accept if J decreases (λ ÷ 3); otherwise reject and increase damping (λ × 5).
4. At most `P.nmpc.maxiter` = 8 iterations, warm-started from the previous solution shifted by one step.

Typical solve time is about 0.5 ms. Speed is not a decision variable; it comes from the shared speed law.

### 4.5 Safety layer

`synthetic_scan.m` ray-casts 719 rays over 240° against the obstacle circles in `P.obstacles`, exactly like the Python bridge. `lidar_safety_filter.m` then:
* replaces invalid readings with 10 m;
* takes the minimum range `dmin` in a ±32° corridor centred on half the steering angle;
* adds repulsion `∓0.35(1/d − 1/0.65)` from the left and right sectors (10°–75°);
* stops (`EMERGENCY_STOP`) at `dmin ≤ 0.28 m`, and scales speed down linearly (`CAUTION`, 15 % floor) below 0.85 m.

The simulation uses the synthetic scan; `run_on_limo` uses the real `/hardware_scan`. Disable it with `P.safety.enabled = false`.

### 4.6 Simulation (`run_simulation.m`)

* The plant is integrated every `P.sim.dt` = 10 ms. The controller runs at its own rate with a zero-order hold in between, like the ROS timers.
* **Steering servo lag:** a first-order lag `P.sim.steer_tau` = 0.05 s between the commanded and actual steering. Set it to 0 for an ideal actuator. With the lag, the high-gain controllers (LQR, LTV-MPC) show visible steering chatter. This is a real effect: it is why LQR has the highest steering rate in the Gazebo results too.
* **Start:** the path start plus `P.sim.start_offset` (0.10 m to the side and 10° of heading by default), to show convergence.
* **End:** an open path ends at its goal; a closed loop ends after `P.sim.laps` laps (time estimate); never longer than `P.sim.t_max`.
* The log has one row per control tick: t, x, y, yaw, v, delta, omega, cte, herr, look-ahead point, dmin, safety status, solve time.

`compute_metrics.m` uses the same definitions as the Python `compare_results.py`: RMS and max cross-track error, RMS heading error, and RMS steering rate.

### 4.7 Running on the LIMO (`run_on_limo.m`)

1. It creates the ROS 2 node `/matlab_limo_controller` on `P.ros.domain_id`, subscribes to `/hardware_odom` and `/hardware_scan`, and publishes on `P.ros.cmd_topic` (`/limo1/cmd_vel` by default, the namespaced LIMO).
2. It waits up to 15 s for odometry (is the bridge running?). Then it publishes the path once on `/reference_path` (`P.ros.publish_path`), so the Python `live_plotter` can show it.
3. Loop at the controller rate: the latest odometry is used as the pose. It is re-zeroed only if `P.ros.rezero_odom` is true; the default is false, so the drawn path is in the PhaseSpace frame. Then come the errors, the controller, the safety filter on the real scan, and `/cmd_vel` with `linear.x = v` and `angular.z = v·tan(delta)/L`.
4. It ends when an open path is finished, `P.ros.duration` elapses (0 = no limit), the live figure is closed, or on Ctrl-C. A **zero Twist is always sent** at the end, via `onCleanup`.
5. It returns the same log as the simulation, plus `metrics`.

Do **not** run a Python controller at the same time: both would publish `/cmd_vel`, and the Python ones would also start on the path MATLAB publishes.

---

## 5. Parameters (`config_limo.m`)

| Field | Default | Meaning |
|---|---|---|
| `L`, `max_steer`, `v_nominal` | 0.20 m, 0.48869 rad, 0.40 m/s | Robot |
| `lab_x`, `lab_y` | [−0.5 8], [−1 6] | Lab grid extents (m) |
| `ref.ds` | 0.01 | Waypoint spacing (m) |
| `ref.speed_gain` | 0.75 | Speed reduction in curves |
| `ref.closed_tol` | 0.15 | Closed-loop detection (m) |
| `ref.search_back`, `ref.search_fwd` | 25, 160 | Nearest-point window (samples) |
| `ref.goal_tol` | 0.05 | Open-path goal tolerance (m) |
| `ref.kappa_window` | 0.10 | Curvature smoothing (m) |
| `smooth.window` | 0.30 | Default smoothing window for drawn paths (m) |
| `pid.*` | kp 1.8, ki 0.05, kd 0.45, kh 1.15, look-ahead 10 | PID |
| `lqr.*` | q_cte 450, q_heading 35, r_steer 0.5, look-ahead 8 | LQR |
| `mpc.*` | N 16, dt 0.04, q 520/35/42, r 0.28, rd 0.45, ki 0.06 | LTV-MPC |
| `nmpc.*` | N 10, dt 0.08, q 40/8, rd 2.5, maxiter 8, 25 Hz | NMPC |
| `safety.*` | enabled, stop 0.28, caution 0.85, repulse 0.65, gain 0.35 | LiDAR safety |
| `obstacles` | [1.970681 0.80659 0.35] | Obstacles `[x y r]` per row, lab frame |
| `sim.dt`, `sim.steer_tau` | 0.01 s, 0.05 s | Plant step, servo lag |
| `sim.start_offset` | [0 0.10 10°] | Initial error [along, left, yaw] |
| `sim.laps`, `sim.t_max` | 1, 120 s | Run length |
| `ros.domain_id` | 0 | ROS_DOMAIN_ID |
| `ros.cmd_topic` | `/limo1/cmd_vel` | The LIMO's velocity topic (`ros2 topic list` on the robot); `/cmd_vel` if it has no namespace |
| `ros.rezero_odom` | false | Path relative to the start pose (true) or PhaseSpace frame (false) |
| `ros.publish_path` | true | Publish the path on `/reference_path` |
| `ros.duration` | 0 | Time limit on hardware (0 = none) |

---

## 6. Verified results

Run in MATLAB R2026a in batch mode:

| Test | Result |
|---|---|
| Figure-8 preset, all four controllers (simulation) | RMS CTE: PID 0.057, LQR 0.031, LTV-MPC 0.045, NMPC 0.032 m. Steering-rate RMS: NMPC 0.21 rad/s (smoothest), LQR 2.57 rad/s. Same ranking as the Gazebo experiments. |
| Jittery scripted free-hand S-curve (4 cm mouse noise) through `draw_path` → smoothing → all controllers | Max curvature after smoothing 3.4 1/m. All four reach the goal (end within 1 cm of the path end). RMS CTE 0.024–0.044 m. |
| Obstacle placed on a straight path | Safety stopped the robot 0.28 m from the obstacle (`EMERGENCY_STOP`) |
| LQR gain vs the Theory worked example | `[-7.071 -3.582]` |
| **ROS 2 end to end:** simulated LIMO publishing PhaseSpace frames → Python bridge → `run_on_limo(ref, 'nmpc', P)` → `/cmd_vel` → simulated LIMO | Reached the end of the S-curve in 27.9 s; RMS CTE 0.013 m; max 0.039 m; 0.63 ms per solve |

Not yet run on the physical LIMO.

---

## 7. Troubleshooting

* **`No /hardware_odom after 15 s`:** start `ros2 run limo_mocap_suite phasespace_bridge`, check that `P.ros.domain_id` equals the terminal's `ROS_DOMAIN_ID`, and do not set `ROS_LOCALHOST_ONLY=1` in the bridge's terminal (MATLAB may not see localhost-only traffic).
* **Robot drives off sideways on hardware:** the path and the pose are in different frames. Keep `P.ros.rezero_odom = false` and draw in lab coordinates, or set it to true and start the robot at the path start facing along the path.
* **"Steering saturated" / large errors on a drawn path:** the curve is tighter than R = 0.38 m. Increase the smoothing window or redraw wider.
* **Dark plots:** all windows use `light_figure` (light theme). If you create your own figures, call `theme(fig, 'light')`.
