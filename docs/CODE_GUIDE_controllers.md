# Controllers and obstacle avoidance: code guide

This guide explains what the four controllers (PID, LQR, LTV-MPC, NMPC) and the two obstacle methods do and how. The shared pieces around them (run manager, planner, logs, launch files) are in `docs/CODE_GUIDE_system.md`. The reasons behind the choices are in `docs/DESIGN_REPORT.md`.

All file names are relative to `src/limo_mocap_suite/limo_mocap_suite/`. A reference `file:line` points to the definition or statement named next to it. The code is the reference for every value in this guide.

## 1. Shared pieces

### 1.1 Robot model

All controllers use the kinematic bicycle model:

$$\dot{x} = v\cos\theta, \qquad \dot{y} = v\sin\theta, \qquad \dot{\theta} = \frac{v}{L}\tan\delta$$

Here $\delta$ is the steering angle at the centre of the axle (bicycle angle), $L$ the wheelbase, $v$ the speed. Kinematic means no tyre slip and no dynamics, which is a fair model at 0.3 to 0.4 m/s.

| Quantity | Value | Where |
|--------------|-----------|-----------------------------------|
| Wheelbase $L$ | 0.20 m | `pid_tracker_node.py:73` (`wheelbase`) |
| `ref_offset` | 0.10 m | `pid_tracker_node.py:74` |
| `max_steer` | 0.4084 rad | `pid_tracker_node.py:75` |
| `v_nominal` | 0.40 m/s | `pid_tracker_node.py:68` |

`max_steer` and `ref_offset` have the same default in the other three nodes (`lqr_tracker_node.py:69-70`, `mpc_tracker_node.py:80-81`, `nmpc_tracker_node.py:76-77`). The node default for `v_nominal` is 0.40 m/s, but the lap suites (`run_laps`) pass 0.30 m/s.

**Tracked point.** The pose from odometry is the midpoint of the two markers, not the rear axle. `ref_offset` is the distance from the rear axle forward to that midpoint (`trajectory_subscriber.py:46`). The bicycle model describes the rear axle. So the NMPC first steps back by `ref_offset` (`nmpc_tracker_node.py:192`) and measures its errors again at the point `ref_offset` ahead of the simulated rear axle (`:203`). The value 0.10 m is an estimate that should be measured on the robot (`hw_check` estimates it).

**Why `max_steer` is 0.4084 rad.** The LIMO driver limits the inner wheel to 28 deg. The equivalent bicycle angle at the centre is smaller: 0.4084 rad, or 23.4 deg. The curvature limit is

$$\kappa_{\max} = \frac{\tan(0.4084)}{0.20} = 2.16\ \mathrm{m}^{-1}, \qquad R_{\min} = \frac{1}{\kappa_{\max}} = 0.462\ \mathrm{m}.$$

### 1.2 Errors

`TrajectorySubscriber.compute_tracking_errors` (`trajectory_subscriber.py:168`) returns the cross-track error, the heading error, the look-ahead point and the nearest index.

- **Nearest point.** It searches a sliding window of table indices from `last_idx - 25` up to `last_idx + 160` (`:175`). The window wraps on a closed path and is clamped on an open one. If the best squared distance is above 2.25 (1.5 m), it searches the whole table (`:183`).
- **Cross-track error** `cte` (`:196`):

  $$e_{ct} = -\sin\theta_r\,(x_r - x) + \cos\theta_r\,(y_r - y)$$

  It is positive when the path lies to the robot's left. A positive error therefore asks for a positive (left) steering, so all gains are positive and are added to the steering.

- **Heading error** (`:197`):

  $$e_{\theta} = \mathrm{wrap}\bigl(\theta_r - \arctan(a\,\kappa_r) - \theta\bigr), \qquad a = \texttt{ref\_offset}$$

  The term $\arctan(a\kappa_r)$ is the angle between the travel direction of the tracked point and the body direction on a curve. With it the error is zero when the robot tracks perfectly. With the plain $\theta_r - \theta$ the error is non-zero on every curve, and the controller settles to a steady cut to the inside of the curve.

- **Look-ahead point.** It is row `nearest + lookahead_pts` (`:188`). It supplies only the reference speed and the curvature for the feed-forward of PID and LQR. The default look-ahead is 10 points for PID (`pid_tracker_node.py:76`), 8 for LQR (`lqr_tracker_node.py:74`) and 9 for LTV-MPC (`mpc_tracker_node.py:89`). The MPC only passes it on, and the NMPC calls `run.tick(..., 5)` (`nmpc_tracker_node.py:279`). Neither uses the value.
- **Goal.** On an open path only: when the nearest point is the last one and the robot is at most 5 cm before the end along the final tangent, `goal_reached` becomes true (`:199-203`).

### 1.3 Path handling

`path_cb` (`trajectory_subscriber.py:73`) turns the poses of `/reference_path` into a table with the columns $[x, y, \theta, v_{ref}, \kappa]$.

| Step | How | Line |
|-------------|----------------------------------------|-------|
| Reject | fewer than 4 poses | 75 |
| Derivatives | `np.gradient` of $x$, $y$, unit tangent, its derivative | 81-91 |
| Yaw | $\theta = \mathrm{atan2}(dy, dx)$ | 94 |
| Curvature | formula below, left turn positive | 97 |
| Speed profile | $v_{ref}$ from section 1.4 | 100 |
| Closed or open | gap first to last point below `max(0.15, 3 x spacing)` | 104-105 |

The curvature is

$$\kappa = \frac{x'y'' - y'x''}{(x'^2 + y'^2)^{3/2}}.$$

The B-spline smoothing of a drawn stroke is not done here. It is done in `interactive_builder.py` before publishing (system guide, section 3.14). Here only finite differences are taken.

`index` (`:135`) maps any index onto the table: modulo $N$ on a closed loop, clamp to $[0, N-1]$ on an open path. All look-ahead and horizon samplers use it, so a horizon cannot run off the end. On an open path the run ends with `goal_reached`. On a closed path indices wrap and the run manager counts laps. `load_table` (`:122`) lets the run manager swap in the way to the path and then the path itself.

### 1.4 Speed law

The reference speed per waypoint is

$$v_{ref} = \frac{v_{nominal}}{1 + 0.75\,\lvert\kappa\rvert}.$$

All four controllers use the same command speed, so the comparison isolates the steering law:

$$v_{cmd} = v_{ref}\,\max\!\Bigl(0.4,\; 1 - 0.5\,\frac{\lvert\delta\rvert}{\delta_{\max}}\Bigr).$$

It is at `pid_tracker_node.py:211`, `lqr_tracker_node.py:231`, `mpc_tracker_node.py:283` and `nmpc_tracker_node.py:261`. Because the steering is clipped to $\delta_{\max}$, the factor never falls below 0.5, so the floor 0.4 never acts. No controller optimises the speed.

### 1.5 From steering angle to Twist

Every controller ends with `RunManager.command(v, steer, ...)` (`run_manager.py:257`):

$$\omega = \frac{v}{L}\tan\delta \quad (\omega = 0 \text{ if } \lvert v\rvert < 10^{-4}).$$

The LIMO base in Ackermann mode inverts this, $\delta = \arctan(\omega L / v)$, so the steering that was computed is the steering the robot gets. The node publishes `/cmd_vel`. The launch files remap it to `/limo1/cmd_vel` with `robot_remappings` (`robot_topics.py:38`). Without a launch file use `-r /cmd_vel:=/limo1/cmd_vel`. Before `command`, `RunManager.shape` (`run_manager.py:242`) clips the steering to $\pm\delta_{\max}$ and scales the speed by the guard factor. `command` clips the steering again.

### 1.6 Common loop

Every controller calls `self.run.tick(...)` first and `self.run.command(...)` last. `RunManager` owns the phases (wait for path, approach, track, finish) and the KILL stop (system guide, section 3.3). The order inside a control step is the same everywhere: tick, steering law, speed law, `run.shape`, `safety.apply` (CBF), `run.command`.

The timer rate is 50 Hz (0.02 s) for PID, LQR and LTV-MPC (`pid_tracker_node.py:115`, `lqr_tracker_node.py:107`, `mpc_tracker_node.py:133`). NMPC runs at 25 Hz (0.04 s, `nmpc_tracker_node.py:125`) because SLSQP is heavier. The parameter `rezero_odom` is `False` by default (`pid_tracker_node.py:77`), so the path is tracked in the absolute MoCap frame.

| Function (`trajectory_subscriber.py`) | Line | What it does |
|----------------------|-----|---------------------------------|
| `__init__` | 43 | Stores `v_nominal`, `ref_offset`; empty table; subscribes `/reference_path` |
| `path_cb` | 73 | Builds the waypoint table from a path |
| `load_table` | 122 | Replaces the tracked table |
| `index` | 135 | Wraps or clamps an index |
| `normalize_angle` | 145 | Wraps an angle to $(-\pi, \pi]$ |
| `compute_tracking_errors` | 168 | $e_{ct}$, $e_\theta$, look-ahead point, nearest index, goal test |

## 2. PID

File `pid_tracker_node.py`. The steering is a curvature feed-forward plus a PID on the cross-track error plus a proportional term on the heading error. It needs no model and is the baseline.

$$\delta_{ff} = \arctan(L\,\kappa_{ref}), \qquad \delta = \delta_{ff} + K_p e_{ct} + K_i\!\int e_{ct}\,dt + K_d\,\dot{e}_{ct} + K_h e_\theta, \qquad \delta \in [-\delta_{\max}, \delta_{\max}]$$

The curvature $\kappa_{ref}$ is taken at the look-ahead point (`:202-210`).

| Parameter | Value | Line |
|-----------------------------------|-------------|-------------|
| `kp_cte` ($K_p$) | 1.8 | 69 |
| `ki_cte` ($K_i$) | 0.05 | 70 |
| `kd_cte` ($K_d$) | 0.45 | 71 |
| `kp_heading` ($K_h$) | 1.15 | 72 |
| `lookahead_pts` | 10 | 76 |

The integral is clamped to $\pm 0.5$ m s (`:197`), so it cannot wind up. The derivative is a plain backward difference, $(e_{ct} - e_{ct,prev})/dt$ (`:198`), not filtered. `_reset_memory` (`:225`) clears both at the start of a run. The time step $dt$ is measured by the run manager.

The feed-forward $\arctan(L\kappa)$ is the steering that holds a steady curve on the bicycle model, because $\tan\delta = L\kappa$. The feedback then only corrects errors. The heading term is needed because the cross-track error alone cannot see that the robot points the wrong way. The gains are tuned by hand, not derived.

In the simulation tests, PID is in the middle for smoothness. Its weak point is that large combined errors add up before the clip: a cross-track error of 0.25 m and a heading error of 0.40 rad give 0.91 rad, which is clipped to the limit.

| Function | Line | What it does |
|-------------|------|-----------------------------------------|
| `__init__` | 64 | Declares parameters; builds helper, CBF layer, run manager, subscribers, 50 Hz timer |
| `quat_to_yaw` | 126 | Quaternion to yaw |
| `odom_cb` | 145 | Stores the pose (optionally re-zeroed); calls `note_odom` |
| `imu_cb` | 169 | Empty placeholder |
| `control_loop` | 189 | tick, PID law, speed law, shape, CBF, command |
| `_reset_memory` | 225 | Clears integral and previous error (called as `on_start`) |
| `main` | 238 | Entry point `pid_tracker` |

## 3. LQR

File `lqr_tracker_node.py`. The error dynamics are linearised around the path, the Riccati equation gives the optimal gain, and the steering is the feed-forward plus $Ke$. The gain is recomputed for the local speed and curvature (gain scheduling).

**State and input.** The state is $e = [e_{ct},\, e_\theta]^T$ (two errors, continuous time). The input $u$ is the steering deviation from the feed-forward. Not in the state: cross-track rate, previous steering, integral.

**Model** (`_solve_lqr_gain`, `lqr_tracker_node.py:176-196`):

$$\dot{e} = A e + B u, \qquad A = \begin{bmatrix} 0 & v \\ 0 & 0 \end{bmatrix}, \qquad B = \begin{bmatrix} 0 \\ \dfrac{v\,(1 + (L\kappa)^2)}{L} \end{bmatrix}$$

The factor $1 + (L\kappa)^2 = \sec^2(\arctan(L\kappa))$ is the linearisation of $\tan$ at the feed-forward steering.

**Solver.** `scipy.linalg.solve_continuous_are(A, B, Q, R)` gives $P$ (`:199`). The gain is $K = R^{-1}B^T P$, row 0. It minimises the infinite-horizon cost $\int (e^TQe + u^TRu)\,dt$.

| Parameter | Value | Line |
|-------------------------------|--------------|--------------|
| `q_cte` | 450 | 71 |
| `q_heading` | 35 | 72 |
| `r_steer` ($R$) | 0.5 | 73 |
| `lookahead_pts` | 8 | 74 |

$Q = \mathrm{diag}(450, 35)$ (`:195`). If the solver throws, the fallback is $K = [2.5,\ 1.2]$ (`:203`).

**Operating point and cache.** $v = v_{ref}$ and $\kappa$ at the look-ahead point. `solve_lqr_gain_cached` (`:248`) memoises the gain on $(v \text{ rounded to } 10^{-3},\ \kappa \text{ rounded to } 10^{-2})$ and clears the cache at 5000 entries (`:251`). So the equation is solved once per distinct rounded pair, not at every tick.

**Control law** (`:227-230`):

$$\delta = \arctan(L\kappa_{ref}) + K_0\,e_{ct} + K_1\,e_\theta, \qquad \delta \in [-\delta_{\max}, \delta_{\max}]$$

$B$ is built with a positive sign on purpose, so the Riccati solution gives a positive $K$ that is added to the feed-forward. This is the same closed loop as the textbook $\delta_{ff} - Ke$ with a negative $B$.

**Gain with the shipped weights** at $v = 0.4$ m/s and $\kappa = 0$ (computed with SciPy): $K = [30.0,\ 9.06]$. A cross-track error of $0.4084 / 30 = 1.36$ cm already saturates the steering. So LQR is close to bang-bang between $\pm\delta_{\max}$ in curves. Lower `q_cte` or raise `r_steer` for smoother steering.

In the simulation tests LQR has the best tracking accuracy (RMS 0.61 cm) and the roughest actuation (steering rate 1.495 rad/s, the highest of the four). It has no look-ahead in time: it reacts to the curvature at one point only.

| Function | Line | What it does |
|-----------------|-----|--------------------------------------|
| `__init__` | 64 | Declares weights, look-ahead; `_gain_cache = {}`; builds helper, CBF layer, run manager, timer |
| `quat_to_yaw` | 118 | Quaternion to yaw |
| `odom_cb` | 135 | Stores the pose |
| `imu_cb` | 159 | Empty placeholder |
| `_solve_lqr_gain` | 176 | Gain $[K_0, K_1]$ for one $(v, \kappa)$ |
| `control_loop` | 220 | tick, cached gain, law, clip, speed law, shape, CBF, command |
| `solve_lqr_gain_cached` | 248 | Memoised gain |
| `main` | 265 | Entry point `lqr_tracker` |

## 4. LTV-MPC

File `mpc_tracker_node.py`. The controller looks 16 steps ahead with a linear model that changes at every step (speed and curvature along the path) and solves the finite-horizon quadratic problem exactly by a backward Riccati recursion. Only the first step is applied (receding horizon).

**State and input.** The state has four components:

$$z = [\,e_{ct},\ \dot{e}_{ct},\ e_\theta,\ u_{prev}\,]^T$$

The input is the steering deviation from the feed-forward. The fourth state stores the last input, so its change can be penalised (slew rate).

| Parameter | Value | Line |
|------------------------------|-----------------------|--------|
| Horizon $N$ (`horizon_steps`) | 16 | 82 |
| Step `mpc_dt` | 0.04 s (0.64 s ahead) | 83 |
| `q_cte` | 520 | 84 |
| `q_dcte` | 35 | 85 |
| `q_heading` | 42 | 86 |
| `r_steer` | 0.28 | 87 |
| `rd_steer` | 0.45 | 88 |
| `lookahead_pts` | 9 | 89 |

The stage weight is $Q = \mathrm{diag}(q_{cte}, q_{dcte}, q_{heading}, r_d)$ (`:230`), the terminal weight is $P_N = 3Q$ (`:232`), and the scalar input weight is $R_{eff} = r_{steer} + r_d = 0.73$ (`:263`). The model speed is floored at $v_k = \max(v_k, 0.15)$ m/s (`:239`).

**Horizon sampling** (`:218-227`). It starts at the nearest index. Each step advances $v_{ref}\,dt/ds$ table indices, where $ds$ is the waypoint spacing. This gives $(v_k, \kappa_k)$ for $k = 0, \dots, N-1$.

**Model per step** (`:248-258`), with $b = v_k\,dt\,(1 + (L\kappa_k)^2)/L$:

$$A_k = \begin{bmatrix} 1 & 0.35\,dt & v_k dt & 0 \\ 0 & 0.20 & v_k & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0 \end{bmatrix}, \qquad B_k = \begin{bmatrix} -0.5\,v_k\,dt\,b \\ -v_k\,b/dt \\ -b \\ 1 \end{bmatrix}$$

The factors 0.35 and 0.20 are empirical damping factors tuned in simulation, not derived. $B$ has the physical negative sign here, so the law is $\Delta u = -Kz$.

**Backward Riccati recursion** (`:262-267`), for $k = N-1, \dots, 0$:

$$S = R_{eff} + B_k^T P B_k, \qquad K_k = \frac{B_k^T P A_k}{S}, \qquad P \leftarrow Q + A_k^T P\,(A_k - B_k K_k)$$

Only $K_0$ is used. Without constraints this is the exact solution of the finite-horizon QP, in $O(N)$ with plain NumPy (the node docstring states below 0.5 ms). No QP solver is needed.

**Applied command** (`:270-279`):

$$\delta_{ff} = 0.45\,f_0 + 0.25\,f_1 + 0.15\,f_2 + 0.10\,f_3 + 0.05\,f_4, \qquad f_k = \arctan(L\kappa_k)$$

$$\delta = \mathrm{clip}\Bigl(\delta_{ff} - K_0 z_0 + 0.06\!\int e_{ct}\,dt\Bigr), \qquad z_0 = [\,e_{ct},\ \dot{e}_{ct},\ e_\theta,\ \delta_{prev} - \delta_{ff}\,]^T$$

The integral is clamped to $\pm 0.5$ m s (`:310`) and its gain 0.06 is hard-coded. $\delta_{prev}$ is the steering that was actually sent (`self.run.steer`, after the CBF). The preview feed-forward starts turning into a corner before the robot reaches it. There are no constraints inside the solver: the steering limit is applied by clipping after the solve. That is the main limit of this controller.

In the simulation tests LTV-MPC has steering rate 0.822 rad/s: smoother than LQR, rougher than NMPC.

| Function | Line | What it does |
|--------------|------|----------------------------------------|
| `__init__` | 75 | Declares parameters and `obstacle_mode`; builds `LinearObstacleMpc`, CBF layer, run manager, timer |
| `quat_to_yaw` | 144 | Quaternion to yaw |
| `odom_cb` | 161 | Stores the pose |
| `imu_cb` | 185 | Empty placeholder |
| `_solve_ltv_mpc` | 207 | Horizon sampling, Riccati recursion, preview, clip, speed law |
| `control_loop` | 302 | tick, $\dot{e}_{ct}$ and integral, constraint branch or Riccati, shape, CBF, command |
| `_constraint_step` | 339 | Step with `obstacle_mode:=constraint` |
| `_reset_memory` | 358 | Clears integral, previous error and steering, obstacle MPC |
| `main` | 374 | Entry point `mpc_tracker` |

## 5. NMPC

File `nmpc_tracker_node.py`. The controller simulates the nonlinear bicycle over 10 steps for a candidate steering sequence, adds up the cost, and lets an optimiser find the best sequence under the steering bounds. The first steering is applied.

**Decision variables.** Only the $N$ steering angles $u_0, \dots, u_{N-1}$. The speed is not optimised: each step uses the reference speed $v_k$ of the path. The command speed comes from the shared speed law with $\lvert u_0\rvert$ (`:261`).

| Parameter | Value | Line |
|------------------|--------------------------------|----------|
| Horizon $N$ | 10 | 78 |
| Step `mpc_dt` | 0.08 s (0.8 s ahead) | 79 |
| `q_cte` | 40 | 80 |
| `q_heading` | 8 | 81 |
| `r_steer` | 0.8, not used in the cost | 82 |
| `rd_steer` | 2.5 | 83 |
| Rate | 25 Hz | 125 |

**Model and cost** (`_cost_function`, `:184-215`). Euler steps of the rear-axle bicycle:

$$x \mathrel{+}= v_k\cos\theta\,dt, \qquad y \mathrel{+}= v_k\sin\theta\,dt, \qquad \theta \mathrel{+}= \frac{v_k}{L}\tan(u_k)\,dt$$

After each step the tracked point is the rear axle plus $a(\cos\theta, \sin\theta)$ with $a$ = `ref_offset`. The errors are taken against reference point $k$ with the same definitions as in section 1.2. The cost is

$$J = \sum_{k=0}^{N-1}\Bigl[\,q_{ct}\,e_{ct,k}^2 + q_h\,e_{\theta,k}^2 + r_d\,(u_k - u_{k-1})^2\,\Bigr], \qquad u_{-1} = \delta_{prev}.$$

There is no terminal cost. $\delta_{prev}$ is the last steering actually sent (after the CBF). The reference for step $k$ is the waypoint one step further on, because cost term $k$ scores the state after step $k$ (`:243`). The start state is the true pose from the odometry.

**Solver: SciPy SLSQP** (`:251-259`):

```
minimize(self._cost_function, self.u_guess, args=(...), method='SLSQP',
         bounds=bounds, options={'maxiter': 6, 'ftol': 1e-3})
```

- Bounds: $-\delta_{\max} \leq u_k \leq \delta_{\max}$ (`:245`), so the limit is exact inside the optimiser.
- Warm start: the last solution is shifted one step left and the new last element is set to $\arctan(L\kappa)$ of the last reference row (`:248-249`).
- Only 6 iterations per tick. Each tick refines the previous plan a little (real-time-iteration style) instead of solving to convergence. The code does not check `res.success`.
- No gradient is given, so SciPy uses finite differences: about $N + 1$ cost evaluations per iteration.

There is no separate feed-forward term. The optimiser finds the steering that follows the curve because the cost compares the simulated path with the reference. The feed-forward appears only in the warm start. The model is the raw bicycle, so $\tan\delta$ is not linearised, which matters near the steering limit.

In the simulation tests NMPC has the smoothest steering (rate 0.196 rad/s) and a slightly larger tracking error (RMS 1.77 cm). `r_steer` is declared but not used, so changing it does nothing.

| Function | Line | What it does |
|--------------|------|----------------------------------------|
| `__init__` | 71 | Declares parameters; builds helper, CBF layer, `NonlinearObstacleMpc`, run manager, 25 Hz timer |
| `quat_to_yaw` | 135 | Quaternion to yaw |
| `odom_cb` | 152 | Stores the pose (no IMU subscription in this node) |
| `_cost_function` | 184 | Cost $J$ of a steering sequence |
| `_solve_nmpc` | 231 | Horizon sampling, warm start, SLSQP, speed law |
| `control_loop` | 278 | tick, constraint branch or `_solve_nmpc` (timed), shape, CBF, command |
| `_constraint_step` | 312 | Step with `obstacle_mode:=constraint` |
| `_reset_memory` | 331 | Clears warm start, previous steering, obstacle MPC |
| `main` | 346 | Entry point `nmpc_tracker` |

### 5.1 Side by side

| | PID | LQR | LTV-MPC | NMPC |
|-------------|------|---------------|--------------|-------------|
| Model | none | linear, 2 states | linear time-varying, 4 states | nonlinear bicycle |
| Look-ahead in time | none | none | 16 x 0.04 = 0.64 s | 10 x 0.08 = 0.8 s |
| Solver | none | `solve_continuous_are` (cached) | backward Riccati recursion | SciPy SLSQP, 6 iterations |
| Steering limit | clip after | clip after | clip after | bound inside the solver |
| Rate | 50 Hz | 50 Hz | 50 Hz | 25 Hz |
| Steering smoothness (sim) | medium | worst | medium | best |

## 6. Obstacles

There are two methods, chosen per run by the parameter `obstacle_mode` of `mpc_tracker` and `nmpc_tracker` (default `'cbf'`, `mpc_tracker_node.py:122`, `nmpc_tracker_node.py:114`). PID and LQR have no look-ahead, so they always use the CBF. No LiDAR is used: the obstacle positions come from the parameter `obstacles` and the latched topic `/hardware_obstacles` (flat list $x, y, r$).

### 6.1 CBF filter

File `obstacle_cbf.py`. Every controller computes its normal $(v, \delta)$. The CBF layer then replaces it with the closest command that keeps the robot out of the known cylinders.

**Barrier.** For the tracked point $p$, obstacle centre $o$ and radius $r$:

$$R = r + r_{robot} + m = r + 0.20 + 0.10 = r + 0.30, \qquad h = \lVert p - o\rVert - R$$

The set $h \geq 0$ is safe (`:202-203`). The point $p$ is a distance $a$ = `ref_offset` ahead of the rear axle, so $\dot p = v\,t + a\,\omega\,n$ with $t$ the heading and $n$ the left normal. The condition $\dot h + \alpha h \geq 0$ with $\alpha$ = `cbf_alpha` = 0.6 (`:384`) is one half-plane in $(v, \omega)$ (`:208`):

$$(d\cdot t)\,v + a\,(d\cdot n)\,\omega + \alpha\,h \geq 0, \qquad d = \frac{p - o}{\lVert p - o\rVert}$$

The offset $a$ lets steering appear in the constraint. At the rear axle only speed would. The constraint is applied only while $h$ is below the trigger distance (`:204`).

**Input set.** The allowed commands are $0 \leq v \leq v_{nom}$ and $\lvert\omega\rvert \leq \kappa_{\max} v$, a triangle, because an Ackermann robot cannot turn on the spot. Here $\kappa_{\max} = \tan(\text{limit})/L$ with the limit `cbf_max_steer` if it is above 0, else `max_steer` (0.4084), so $\kappa_{\max} = 2.16\ \mathrm{m}^{-1}$ (`:136`, `:389`). The safe set is this triangle cut by one half-plane per near obstacle. It contains $(0,0)$ while $h \geq 0$, so stopping is always possible. The safe command is the point closest to the reference $(v_{ref}, \omega_{ref})$ for the cost

$$(v - v_{ref})^2 + w_s\,(\omega - \omega_{ref})^2, \qquad w_s = 0.15.$$

Since $w_s < 1$, changing $\omega$ is cheaper than changing $v$. It is solved in closed form by polygon clipping (`_project`, `:279`), with no solver.

**Why two layers.** A pure distance barrier reacts too late for a robot that cannot turn on the spot: it arrives at the boundary pointing inwards and has to stop. So an approach layer starts the turn early, and the barrier remains the actual guarantee.

**Approach layer, step by step** (`CbfFilter.filter`, `:188`):

1. **Trigger distance** (`trigger_distance`, `:159`). A head-on turn of radius $\rho$ clears a circle of radius $R$ only if it starts at a centre distance of at least $\sqrt{R^2 + 2R\rho}$. Here $\rho = 1/(0.85\,\kappa_{\max})$, which uses 85 % of the steering limit, and 0.25 m is added for servo lag. Measured from the clearance circle:

$$d_{trig} = \sqrt{R^2 + 2R\rho} - R + 0.25$$

   With the defaults this is 0.63 m for $r = 0.15$ m and 0.66 m for $r = 0.35$ m.

2. **Collision cone.** Its half-angle is $\arcsin(\min(1, R/\text{dist}))$ (`:222`). The bearing is the direction to the obstacle relative to the heading.
3. **Side commitment.** On the first tick in range, one side is chosen per obstacle and kept in `self.side[i]` ($+1$ left, $-1$ right) until the obstacle leaves the trigger range (`:214-221`, release `:235`). The choice, in priority order:
    - From the path ahead: `_path_side` (`:256`) finds the path point nearest the obstacle centre within the next 3 m (`PATH_AHEAD = 3.0`, `:35`) and takes the sign of the cross product of the path direction and the vector to the centre. Obstacle left of the path: pass on the right (`:216`). Obstacle right of the path: pass on the left.
    - If the centre is within 2 cm of the path line or there is no path: obstacle left of the heading (bearing above 0.02 rad) means turn right, otherwise left (`:218`).
    - Dead ahead: the direction of the nominal command (`:220`).

   The side is not taken from the heading alone, because a few degrees of heading error flipped the choice when the obstacle was just off the path, and the robot went round the outside. Only the next 3 m are used, because further on the same path can pass the same obstacle again from the other side (approach plus track prefix).

4. **Violation and hysteresis.** `violation = cone + CONE_HYST + side * bearing` if $\lvert\text{bearing}\rvert < \pi/2$, else 0 (`:225`). `CONE_HYST` = 0.08 rad (`:36`) makes the cone wider, so the status does not flip between CLEAR and ACTIVE at the cone edge.
5. **Reference turn.** For the worst obstacle (`:228-231`): `strength = min(1, violation/0.25)`, `w_turn = side * kappa_max * v_nom * strength`, `w_ref = max(w_nom, w_turn)` for a left pass or `min` for a right pass, and `v_ref = v_nom (1 - 0.5 strength)`. The reference turns toward the committed side, at most full lock, and slows by up to half.
6. **Projection** (`_project`, `:279`): clip the triangle with each half-plane (Sutherland-Hodgman, `_clip`), then take the closest point to the reference, or the reference itself if it is inside. The steering is $\delta = \arctan(L\omega/v)$, clipped to $\pm\delta_{\max}$ (`:245`).

**Why the path is a straight tangent line, then a curve.** The reference turn pushes the heading only to the edge of the cone (plus 0.08 rad), not further. The robot therefore drives a straight tangent to the safety circle and then curves around it. It reacts late because it has no plan. Constraint mode (6.2) plans earlier and more smoothly.

**Statuses.**

| Status | Meaning |
|-------------|-----------------------------------------------|
| `NO_OBSTACLES` | List empty, clearance infinite |
| `CLEAR` | No obstacle in range, or $v_{nom} \leq 10^{-4}$, or the nominal command already satisfies every half-plane |
| `CBF_ACTIVE` | The command was modified |
| `CBF_STOP` | Projection empty or speed below $10^{-3}$: speed 0, steering unchanged |

The status and the clearance (smallest distance to a cylinder edge) go to the CSV.

**Steering rate limit** (`ObstacleCbfLayer.apply`, `:429`). The clean projection can jump the steering from tick to tick. While an obstacle is "near", or for `cbf_hold` = 0.6 s after the filter last acted (`CBF_ACTIVE` or `CBF_STOP`, `:439`), the steering may change by at most `cbf_steer_rate` $\cdot\, dt$ = 2.0 rad/s $\cdot\, dt$ (`:450`). The time step is capped at 0.1 s (`:449`). At 50 Hz that is 0.04 rad per tick. "Near" means a distance to the clearance circle of at most trigger distance + 0.3 m (`:443`). The limit is not applied when the speed is about 0 (`:446`). Outside this, the controller's steering passes through unchanged.

**Guarantee.** For the model, a safe command exists whenever $h \geq 0$, because stopping is in the set. Delay, servo lag and MoCap noise can eat the margin, which is why `safety_margin` exists. An obstacle that MoCap never saw does not exist for the filter.

| Function (`obstacle_cbf.py`) | Line | What it does |
|--------------------|-----|-----------------------------------|
| `parse_obstacles` | 47 | Reads the obstacle list (None, string, list); checks multiple of 3, radius > 0 |
| `parse_marker_groups` | 67 | Reads groups of 4 marker ids |
| `circle_from_markers` | 88 | Cylinder from rim markers: centre = mean, radius = mean distance |
| `declare_flexible_parameter` | 102 | Declares a parameter of any type (empty list allowed) |
| `CbfFilter.__init__` | 139 | Stores robot and CBF settings; $\kappa_{\max}$; `side = {}` |
| `CbfFilter.trigger_distance` | 159 | Distance at which to start turning |
| `CbfFilter.filter` | 188 | Main step; returns $(v, \delta, \text{clearance}, \text{status})$ |
| `CbfFilter._path_side` | 256 | $+1$ / $-1$ / 0: obstacle left / right / unknown relative to the path |
| `CbfFilter._project` | 279 | Closest safe $(v, \omega)$ |
| `CbfFilter._clip` | 308 | Clips a convex polygon by a half-plane |
| `CbfFilter._closest_on_segment` | 328 | Nearest point on a segment |
| `CbfFilter._inside` | 342 | Point in convex polygon |
| `ObstacleCbfLayer.__init__` | 379 | ROS side: parameters, latched subscription to `/hardware_obstacles` |
| `ObstacleCbfLayer._cb` | 411 | Stores the obstacle list from the topic |
| `ObstacleCbfLayer.apply` | 429 | Frame transform, filter, hold timer, rate limit |
| `ObstacleCbfLayer._path_ahead` | 465 | The next 3 m of the path from the robot's index |

The layer declares `robot_radius` 0.20, `safety_margin` 0.10, `cbf_alpha` 0.6, `cbf_max_steer` 0.0 (0 means use `max_steer`), `cbf_hold` 0.6 and `cbf_steer_rate` 2.0 (`:382-387`).

### 6.2 Constraint mode

File `mpc_obstacles.py`. It is used when `obstacle_mode:=constraint`. The CBF filter is then not used: the MPC plans around the obstacles itself. Speed stays outside the optimiser, so only the steering avoids obstacles. The module constants are at `mpc_obstacles.py:34-37`.

| Constant | Value | Meaning |
|---------------|---------|------------------------------------|
| `N` | 20 steps | Horizon |
| `DT` | 0.15 s | Step; $20 \times 0.15 = 3$ s look-ahead |
| `ACTIVE_RANGE` | 2.0 m | Obstacles further away are ignored |
| `MODEL_MARGIN` | 0.10 m | Extra keep-out in the linear QP only |

The normal horizons (0.64 s and 0.8 s) end before the robot could turn away, hence 3 s here. The keep-out distance is `robot_radius + safety_margin` = 0.30 m. In this mode the horizon and weights of the normal solvers (`horizon_steps`, `mpc_dt`, `q_cte`, ...) are not used. The two classes use their own defaults given below.

**LinearObstacleMpc** (`:103`, used by LTV-MPC). The model is about the path, with $e$ the sideways offset (left positive) and $\vartheta$ the heading offset:

$$e_{k+1} = e_k + v\,DT\,\vartheta_k, \qquad \vartheta_{k+1} = \vartheta_k + \frac{v\,DT\,\sec^2(\delta_{ff})}{L}\,u_k$$

Here $u$ is the steering minus the feed-forward. Both $e$ and $\vartheta$ are linear in $u$, so the cost is quadratic. The start is $e_0 = -e_{ct}$ and $\vartheta_0 = -e_\theta$ (`:199`).

- **Cost:** $q_e e^2 + q_\vartheta \vartheta^2 + r u^2 + r_d (\Delta\delta)^2$ with $q_e = 30$, $q_\vartheta = 4$, $r = 0.5$, $r_d = 6$ (`:111`).
- **Obstacle as a bound on $e$** (`_bounds`, `:153`). With $R = r + \text{keep\_out} + \text{MODEL\_MARGIN} = r + 0.40$, for each step $k$ where the obstacle is within $R$ along the path tangent ($\lvert ds\rvert < R$) and not too far sideways ($\lvert\text{offset}\rvert < R + 0.25$, `:165`): $w = \sqrt{R^2 - ds^2}$. Passing on the left gives $e \geq \text{offset} + w$ (`:170`), passing on the right gives $e \leq \text{offset} - w$ (`:172`).
- **Side** (`_side`, `:135`): chosen once per obstacle and remembered. If the centre is on the right of the path (offset $\leq 0$), pass left, else pass right. The model is linear, so an obstacle can only be a bound on the offset, and a side must be chosen first.
- **Slack.** The constraints are $e + s \geq \text{lower}$ with $s \geq 0$ and a cost $2\cdot 10^5\, s^2$ (`slack_w`, `:217`). The slack keeps the QP always solvable, and its value tells how far the best plan falls short.
- **Solver:** `scipy.optimize.minimize(..., method='SLSQP')` with the exact gradient, `maxiter` 30, `ftol` $10^{-5}$ (`:238`). It is a convex QP. Bounds on the steering: $-\delta_{\max} - \delta_{ff} \leq u \leq \delta_{\max} - \delta_{ff}$ (`:235`). Warm start: the previous solution shifted by one step (`:236`).
- **Output:** steering $\mathrm{clip}(\delta_{ff,0} + u_0)$ and the slack in metres.

**NonlinearObstacleMpc** (`:253`, used by NMPC).

- It rolls the same bicycle forward (`_rollout`, `:285`) over $N = 20$ steps of 0.15 s. The cost is the NMPC cost with $q_{ct} = 40$, $q_h = 8$, $r_d = 2.5$ (`:261`).
- **Constraint:** the true distance from the tracked point to each cylinder at every step must be at least $r + \text{keep\_out}$ (`gaps`, `:348`). These are SLSQP inequality constraints, only for obstacles within 2 m. No extra margin is added.
- **Solver:** SLSQP, `maxiter` 6, `ftol` $10^{-3}$ (`:355`), steering bounds $\pm\delta_{\max}$, finite-difference gradients, warm start by shifting.
- **Side by initial guess.** The problem is nonconvex, so SLSQP finds only the nearest local solution. When an obstacle first comes into range, `_guess` (`:308`) seeds an S-shaped steering sequence: find the reference step $k$ nearest the obstacle, `side = +1` if the centre is on the right (offset $\leq 0$, pass left), then `u[:turn] = side*bend` and `u[turn:2*turn] = -side*bend` with `bend = 0.8 * max_steer` and `turn = max(2, min(k//2, N//3))` (`:317-320`). The obstacle is remembered in `seen`, so this is done once.
- **Output:** the first steering and `short`, how far the plan's closest approach falls short of the keep-out (m).

**Speed rule** (`speed_for_plan`, `:371`):

| Condition | Speed | Status |
|---------------------------|------------------------|---------|
| shortfall $\leq 0.01$ m, clearance $< 0.9$ m | unchanged | `MPC_ACTIVE` |
| shortfall $\leq 0.01$ m, clearance $\geq 0.9$ m | unchanged | `MPC_CLEAR` |
| shortfall above 0.01 m, clearance $< 0.08$ m | 0 | `MPC_STOP` |
| shortfall above 0.01 m, otherwise | $v\,\max(0.3,\ 1 - \text{shortfall}/0.2)$ | `MPC_ACTIVE` |

The thresholds 0.9 m and 0.08 m are `keep_out + 0.6` and `0.08 + 0.5 (keep_out - 0.3)` with the default keep-out of 0.30 m.

| Function (`mpc_obstacles.py`) | Line | What it does |
|----------------------|-----|---------------------------------|
| `world_obstacles` | 47 | Obstacles in the controller frame |
| `sample_reference` | 64 | 20 reference rows, 0.15 s apart at reference speed |
| `clearance` | 82 | Smallest distance to a cylinder edge |
| `LinearObstacleMpc.__init__` | 111 | Weights, side memory, warm start |
| `LinearObstacleMpc.reset` | 122 | Clears sides and warm start |
| `LinearObstacleMpc._side` | 135 | Pass side per obstacle, remembered |
| `LinearObstacleMpc._bounds` | 153 | Lower and upper bound on $e$ per step |
| `LinearObstacleMpc.solve` | 189 | Builds the QP with slack; SLSQP |
| `NonlinearObstacleMpc.__init__` | 261 | Weights, warm start, `seen` set |
| `NonlinearObstacleMpc.reset` | 272 | Clears warm start and `seen` |
| `NonlinearObstacleMpc._rollout` | 285 | Tracked-point positions for a steering sequence |
| `NonlinearObstacleMpc._guess` | 308 | S-shaped first guess away from the centre |
| `NonlinearObstacleMpc.solve` | 333 | Cost plus distance constraints; SLSQP |
| `speed_for_plan` | 371 | Slows or stops if there is no safe plan |

### 6.3 CBF versus constraint mode

| | CBF (default) | Constraint mode |
|----------|--------------------------|------------------------|
| Where | Filter after the controller | Inside the MPC |
| Controllers | All four | LTV-MPC and NMPC |
| Look-ahead | None: reacts within about 0.65 m | 3 s, obstacles within 2 m |
| Guarantee | Yes, for the model | Soft: slack, then slow or stop |
| Extra | Steering rate limit 2 rad/s | Speed rule `speed_for_plan` |

The CBF takes the side from the path ahead and holds it. LTV-MPC takes it from the side of the path on which the centre lies, and NMPC from the initial S guess. The CBF drives a straight tangent to the safety circle and then hard steering at the last moment. Constraint mode is expected to turn earlier and more smoothly, but no hardware result exists yet. `obstacle_lap.launch.py compare_modes:=true` runs both MPCs in both modes.

## 7. Likely questions

**Which solver does the NMPC use, and why?**
SciPy `minimize` with SLSQP (`nmpc_tracker_node.py:251`). It is part of SciPy on Ubuntu 20.04 with ROS 2 Foxy, so there is no extra dependency. It handles the steering bounds exactly, copes with the nonconvex $\tan$ and $\cos$ cost, and supports nonlinear inequality constraints for the obstacle mode. The problem is small (10 variables). Faster alternatives such as CasADi with IPOPT or acados would need installation on the lab PC and were not benchmarked.

**How many iterations does the NMPC use? Does it converge?**
At most 6 iterations with `ftol` $10^{-3}$, warm-started from the shifted last solution. Each tick refines the plan; it is not solved to convergence.

**What is the state of the LQR?**
Two states, cross-track error and heading error, $e = [e_{ct}, e_\theta]$. The input is the steering deviation from the feed-forward. The model is continuous with $A = [[0, v], [0, 0]]$ and $B = [0,\ v(1 + (L\kappa)^2)/L]^T$.

**How is the LQR gain computed?**
With `scipy.linalg.solve_continuous_are` and $K = R^{-1}B^TP$, for $Q = \mathrm{diag}(450, 35)$, $R = 0.5$ and the local speed and curvature. The result is cached. On a straight at 0.4 m/s the gain is $[30.0,\ 9.06]$.

**Why does the LQR saturate so much?**
The cross-track gain is 30 rad/m, so 1.4 cm of error uses the whole 0.4084 rad. $Q$ is large compared with $R$. Lower `q_cte` or raise `r_steer` to smooth it.

**Why the feed-forward $\arctan(L\kappa)$?**
It is the steering that holds a curve of curvature $\kappa$ on the bicycle model ($\tan\delta = L\kappa$). The feedback then only corrects errors.

**What are the state and solver of the LTV-MPC?**
The state is $[e_{ct}, \dot{e}_{ct}, e_\theta, u_{prev}]$. It uses a backward Riccati recursion over 16 steps of 0.04 s and applies only $K_0$. It is unconstrained, and the steering is clipped afterwards.

**Why a Riccati recursion and not a QP solver for the LTV-MPC?**
Without constraints the QP has a closed-form solution. The recursion is $O(N)$ in plain NumPy and fast enough for 50 Hz. A QP solver would add nothing.

**What do the factors 0.35 and 0.20 in the LTV-MPC matrix $A$ mean?**
They are empirical damping factors tuned in simulation, not derived.

**Why does the NMPC run at 25 Hz and the others at 50 Hz?**
SLSQP is heavier (comment at `nmpc_tracker_node.py:124`), and the horizon is 10 steps of 0.08 s.

**Is the speed optimised?**
No. All controllers use $v_{ref}\max(0.4,\ 1 - 0.5\lvert\delta\rvert/\delta_{\max})$, so the comparison is about steering only.

**Why is `max_steer` 0.4084 rad and not 28 deg?**
28 deg is the inner-wheel limit of the driver. The bicycle model needs the centre angle, which is 0.4084 rad (minimum radius about 0.46 m).

**What is `ref_offset`?**
The distance of the tracked marker midpoint ahead of the rear axle, 0.10 m. The heading error subtracts $\arctan(a\kappa)$, and the NMPC simulates the rear axle and measures at the offset point.

**Why is the heading error $\theta_r - \arctan(a\kappa) - \theta$?**
So that it is zero when tracking perfectly on a curve. Otherwise the controller cuts the inside of every curve with a steady offset proportional to the curvature.

**What is the sign of the cross-track error?**
Positive when the path is to the left of the robot, so positive gains steer left.

**How does the CBF decide the side?**
Once per obstacle, when it enters the trigger range: from which side of the next 3 m of the path the obstacle centre lies. An obstacle left of the path is passed on the right. If the centre is within 2 cm of the path or there is no path, the heading decides. If the obstacle is dead ahead, the direction of the nominal turn decides. The side is kept until the obstacle leaves the range.

**What are the barrier and the constraint?**
$h = \lVert p - o\rVert - (r + 0.20 + 0.10)$ and $\dot h + 0.6\,h \geq 0$, which is one half-plane $(d\cdot t)v + a(d\cdot n)\omega + 0.6\,h \geq 0$ in $(v, \omega)$.

**Why a collision-cone layer on top of the barrier?**
A pure distance barrier reacts too late for a car-like robot: it arrives pointing inwards and has to stop. The cone layer starts turning about 0.65 m from the safety circle (`trigger_distance`), and the barrier gives the guarantee.

**What is the hysteresis?**
`CONE_HYST` = 0.08 rad is added to the cone half-angle so that the status does not toggle between CLEAR and ACTIVE at the cone edge.

**What does the steering rate limit do?**
Near an obstacle, and for 0.6 s after the CBF acted, the steering may change at most 2.0 rad/s times $dt$ (0.04 rad per 50 Hz tick). It removes jumps from the projection.

**What are the statuses?**
CBF: `NO_OBSTACLES`, `CLEAR`, `CBF_ACTIVE`, `CBF_STOP` (no safe forward motion, speed 0). Constraint mode: `MPC_CLEAR`, `MPC_ACTIVE`, `MPC_STOP`.

**How does constraint mode avoid obstacles?**
With a 3 s look-ahead. LTV-MPC turns the obstacle into a bound on the sideways offset (side chosen once) and solves a QP with slack by SLSQP. NMPC constrains the true distance to each cylinder at every step (SLSQP, S-shaped initial guess away from the centre).

**What if there is no safe plan in constraint mode?**
The shortfall is positive. The robot slows with $v\max(0.3,\ 1 - \text{shortfall}/0.2)$ and stops if the clearance is below 0.08 m.

**Which controller is best?**
In simulation LQR has the lowest tracking error (RMS 0.61 cm), NMPC the smoothest steering (0.196 rad/s) and LTV-MPC lies in between; LQR saturates the steering the most. All results come from simulation. Hardware results are pending.
