# Design report: LIMO Ackermann tracking, the choices and the evidence

This report says what was decided, why, what the evidence is, and which limits remain. The guides say what the code does and how: `docs/CODE_GUIDE_controllers.md` for the controllers and obstacle avoidance, `docs/CODE_GUIDE_system.md` for everything around them. Numbers and parameter names are not repeated here when a guide has them.

**Status of the evidence.** Everything quoted here comes from simulation: `sim_limo`, the Gazebo stand-in, and desk tests with a stub of `phasespace_msgs`. No tracking suite has run on the real LIMO yet (`HANDOFF_hardware.md`, `docs/LAB_GUIDE.md`). Where a document and the code disagree, the code wins (Appendix A).

File names are in `src/limo_mocap_suite/limo_mocap_suite/` unless a path is given.

## 1. Goal and setup

**Goal.** Compare four path-tracking controllers on a real Ackermann robot with one repeatable test, and check how each one copes with a tight curve and with obstacles.

| Item | Value |
|-------------|-----------------------------------------------|
| Robot | AgileX LIMO in Ackermann mode; wheelbase 0.20 m, track 0.172 m |
| Steering limit | 0.4084 rad at the centre; tightest radius 0.46 m, curvature 2.16 1/m (section 4.6) |
| Positioning | PhaseSpace mocap, ROS frame (+x forward, +y left, yaw counter-clockwise) |
| Markers | Marker 0 front, marker 1 rear; pose is the midpoint; heading from marker 1 to marker 0 |
| Tracked point | Marker midpoint, `ref_offset` 0.10 m ahead of the rear axle (an estimate, to be measured) |
| Commands | `Twist` with $v$ and $\omega = v\tan\delta/L$ on `/limo1/cmd_vel` |
| Control rate | 50 Hz for PID, LQR, LTV-MPC; 25 Hz for NMPC |

The four controllers:

| Controller | Math | Solver |
|------|-------------------------|-----------------------------|
| PID | Error feedback plus curvature feed-forward | None (direct formula) |
| LQR | Linear error model, infinite horizon | Continuous Riccati equation (`scipy.linalg.solve_continuous_are`) |
| LTV-MPC | Linear model changing along the path, finite horizon | Backward Riccati recursion (closed form) |
| NMPC | Nonlinear bicycle model, finite horizon | `scipy.optimize.minimize`, SLSQP |

**Common structure.** All four share `run_manager.py` (approach, laps, kill, logs), `trajectory_subscriber.py` (nearest point, errors) and `obstacle_cbf.py` (obstacle safety). They also share one speed law,

$$v = v_{ref}\,\max\!\Bigl(0.4,\ 1 - 0.5\frac{\lvert\delta\rvert}{\delta_{\max}}\Bigr), \qquad v_{ref} = \frac{v_{nominal}}{1 + 0.75\lvert\kappa\rvert},$$

so the comparison isolates the steering law. The lap suites run at $v_{nominal}$ = 0.30 m/s (`run_laps.launch.py`); the node default is 0.40 m/s.

## 2. Test design

### 2.1 Why a fixed capsule track

| Reason | Detail |
|---------------|---------------------------------------------|
| Repeatable | The same points every run (`track_tools.capsule()`, 1 cm spacing); a drawn stroke differs each time |
| Checkable | Curvature is exact: 0 on the straights, $1/R$ on the arcs. A drawn stroke needs smoothing and is noisy: in a MATLAB test a stroke with 4 cm jitter gave a peak curvature of 100 1/m before smoothing and 3.4 1/m after (CHANGELOG 3.10) |
| Fair | With a fixed $R$ the steering demand is a known number (section 2.3); a drawn stroke may be tighter than the robot can turn |
| Same for everyone | One track for the four controllers, the sim and the robot |

The track is a running track: two straights of 4.0 m joined by two half circles; point 0 is the start. Hand drawing still works (`hw_tracking.launch.py` without `track_radius`) but is not used for the comparison.

A weakness to state if asked: curvature jumps from 0 to $1/R$ at each joint, with no clothoid transition. That is a hard step for the steering, which is useful as a test, but it is not how a road is built.

### 2.2 Size and place in the lab

The lab (mocap) area is x from -0.5 to 7.0 m and y from -0.5 to 3.0 m. The track has $R$ = 1.0 m, straights of 4.0 m and centre (3.25, 1.25), so it spans x 0.25 to 6.25 m and y 0.25 to 2.25 m. That leaves 0.75 m to every edge, for drift or for swerving round an obstacle. `capsule_message()` refuses a track that does not fit ("Track does not fit the lab"), and `run_laps` checks this before anything moves.

![The lab area, the R1 lab track (CCW), the R0.5 add-on, the race starts and the obstacle spots. ROS frame: +x forward, +y left.](figures/lab_track.png)

### 2.3 Why $R$ = 1.0 m

The steering needed for a steady turn is $\delta = \arctan(L/R)$. With $L$ = 0.20 m and the limit 0.4084 rad:

| $R$ (m) | Steering needed (rad) | Share of the limit |
|----------|--------------------------|-----------------------|
| 1.0 | 0.197 | 48 % |
| 0.5 | 0.381 | 93 % |
| 0.462 | 0.408 | 100 % |
| 0.4 | 0.464 | 114 % (impossible) |

Evidence from the earlier simulation study (`evaluation/lap_test_20261105_080758/report.md`, last-lap values):

| Radius | RMS error, four controllers | Share of the lap at the limit |
|-------|--------------------------|---------------------------|
| R1 | 0.18 to 0.78 cm | 0 to 14 % |
| R0.5 | 0.44 to 2.56 cm | 59 to 77 % |
| R0.4 | 10.0 to 11.5 cm | 70 to 85 % |

So $R$ = 1.0 m is well inside the limit: the controllers can differ in quality, not in whether the robot can physically do it. $R$ = 0.4 m is beyond the limit, and every controller fails by about the same amount, which says little about the controller. That study used an older track centre, 2 laps and a kinematic stand-in, so treat the numbers as the shape of the result, not as predictions.

### 2.4 Start, direction, laps

| Choice | Rule |
|-----------------------|-------------------------------------|
| Start in the middle of a straight | Curvature is 0 at the start, there are 2 m to reach speed before the first curve, and the lap boundary falls on a straight |
| Start side `auto` | Top straight, unless the robot is below the centre line ($y$ < 1.25 m), then the bottom; decided right before each run from `/hardware_odom` |
| Direction CCW | A convention (left turns). A CW run checks for left-right differences in the real steering |
| 3 laps | Lap 1 includes settling after the approach; laps 2 and 3 show whether the result repeats |
| First metre of lap 1 | Not scored, which removes the launch transient |

A lap is counted by forward progress along the path, not by crossing the start line (system guide, 3.3). Stopping or wobbling at the start line cannot add a lap. The old study used 2 laps; the suites now default to 3.

### 2.5 One controller per run, or all four in turn

| Suite | What it does | Log |
|------------------|--------------------------|----------------|
| `record_lap.launch.py controller:=X` | One controller, 3 laps | `evaluation/record_<X>_<stamp>/` |
| `compare_lap.launch.py` | The four in turn, 3 laps each, `buffer_time` 10 s between | `evaluation/compare_<stamp>/` |

Each controller is a separate process started fresh with its memory cleared. Between two of them the robot is held at zero command, then the next one drives itself back to the start with the same planner. Same start rule, same track, same speed law, so the numbers are comparable. Limits: robot, battery and floor are not identical between runs, and order effects are not controlled. If a difference is small, repeat the suite and swap the order (`controllers:=...`). If a run ends with anything other than `FINISHED` or `TIME_LIMIT`, the whole suite stops, so a KILL cancels the rest.

### 2.6 R0.5 add-on

`r05_lap.launch.py` runs the same suite with $R$ = 0.5 m (same straights and centre). That needs 93 % of the steering limit, so every controller works near saturation. It shows which controller copes at the edge of what the robot can do, and it is kept apart from the main suites. The track spans x 0.75 to 5.75 m and y 0.75 to 1.75 m.

## 3. Getting to the start

### 3.1 The problem

The robot can start anywhere in the mocap area. It is a car: it cannot turn on the spot and the planner does not reverse. Tracking the path directly from far away does not work: the look-ahead point can sit inside the turning circle, the steering saturates, and the robot can orbit without arriving (CHANGELOG 3.17).

### 3.2 Why Dubins paths

A Dubins path is the shortest path between two poses for a forward-only vehicle with a minimum turning radius: arcs and one straight. `approach_radius` is 0.60 m, larger than the 0.46 m limit. That curve needs $\arctan(0.2/0.6)$ = 0.32 rad, which is 79 % of the limit, leaving margin for servo error. `approach_speed` is 0.25 m/s.

### 3.3 Decision history

| Step | What was tried | What went wrong |
|----|--------------------------|------------------------------|
| 1 | Track the path directly | Steering saturated, the robot could orbit |
| 2 | Shortest Dubins path to the start pose | Fine near the path; from a wrong heading it loops, often near an edge |
| 3 | Shortest route staying 0.25 m inside the lab edges | A route that just touches the edge leaves the mocap area when the robot overshoots |
| 4 | Score-based choices: quickest drive, closest merge | Plans were hard to predict; a pure closest merge rode up to 12 m round the track |
| 5 | One fixed merge rule, like a person joining a road | Final (system guide, 3.3, and CHANGELOG 3.21) |

The final rule in one sentence: find the track point in front of you going your way, merge 1 m further on, follow the track to the start; no loops, stay inside the lab, clear of obstacles; if that fails, move the merge point on, and as a last resort drive straight to the start. The rule, its constants and its edge levels (0.25 m inside, the lab edge, 0.30 m past it, else refuse) are in the system guide, 3.3 and 3.4.

A sweep of 840 start poses (every 0.5 m, 8 headings) compares the old and the new rule:

| | Old rule (Dubins to the start pose) | Fixed merge rule |
|-----------------|-------------------|------------------------|
| Median turning | 253 deg | 142 deg |
| Loops | common | 4 |
| Worst excursion outside the lab | 1.30 m | 0.27 m (poses 0.25 m from an edge, facing out) |

With the final rule, 568 poses merge onto the track, 37 merge onto the start itself, 187 go by the shortest curve straight to the start, and 48 are refused (all 25 cm from an edge, facing out). The sweep is `tools/planner_check.py`; the drawing is `docs/figures/fixed_merge_rule.png`.

![The fixed merge rule from six start poses (drawn by `tools/planner_check.py`). Arrow = robot, star = start, orange = merge curve, blue = ride along the track. Last panel: obstacle run.](figures/fixed_merge_rule.png)

### 3.4 Why a fixed rule instead of scoring

A fixed rule is predictable (you can say where the robot will go before it moves), explainable in one sentence, testable by sweep, and matches what people expect of a car. Scoring depends on weights, so a small change in pose can flip the plan. The cost is that the fixed rule is not the quickest. It was chosen with the owner after looking at drawings of the alternatives.

**Why refuse at the edge.** Outside the mocap area the robot is lost: no pose, so no control. In a rehearsal a robot 0.3 m from an edge, facing out, needed 0.6 to 0.7 m outside whichever way it turned, so it cannot be done within the 0.30 m spill. The advice (`docs/LAB_GUIDE.md`) is to place the robot at least 0.6 m inside the edges and not facing straight out.

**Limits.** The approach is planned once at the start; with the guard off there is no re-plan. Loops are forbidden in the merge but may occur in the last-resort fallback.

## 4. Controllers

They span the common ways to do tracking, so the comparison tells something: a simple baseline, an optimal linear controller, a controller that looks ahead along a changing model, and one that uses the full nonlinear model. Parameters and equations are in the controllers guide.

### 4.1 PID

The baseline: a curvature feed-forward plus PID on the cross-track error plus a proportional heading term. It has no model of the robot and is what any user would write first, so it shows what the other three add. In the simulation study it has the highest error at R1 and R0.5 (0.78 and 2.56 cm in the last lap).

### 4.2 LQR

Optimal state feedback on the two errors with cost weights $Q = \mathrm{diag}(450, 35)$ and $R = 0.5$. The gain comes from the continuous Riccati equation, solved for the current $(v_{ref}, \kappa)$ and memoised, so a straight or a constant arc costs one solve. It is in the comparison because it is the standard optimal controller and needs no tuning of separate gains.

The known weakness is a high gain on the cross-track error: at 0.4 m/s the servo saturates at 1.36 cm of error. In the R1 study LQR spent 13.9 % of its last lap at the limit with a peak steering of 100 %, while the others spent 0 %. So its RMS is good but it has no steering reserve.

### 4.3 LTV-MPC

Finite-horizon optimal control of a model that changes along the path: 16 steps of 0.04 s (0.64 s). A backward Riccati recursion gives the first gain, and the feed-forward is a weighted preview of the curvature over five steps.

**Why a Riccati recursion and not a QP solver.** With no inequality constraints the recursion gives the exact solution of the QP in $O(N)$, so it runs at 50 Hz in well under a millisecond (the node docstring states below 0.5 ms; no report measures it). There is no iteration and no failure case.

Be straight about what it is:

- There are no constraints inside the solver. The steering limit is applied by clipping afterwards.
- Two entries of the model matrix (0.35 and 0.20) are empirical damping factors tuned in simulation, not derived.
- So it is a time-varying LQR with preview, not a constrained MPC. In obstacle constraint mode a different, constrained solver is used (section 5.8).

### 4.4 NMPC

Finite-horizon optimal control with the full nonlinear bicycle model, so $\tan\delta$ is not linearised. This matters near the steering limit, where the linear model is least accurate. The horizon is 10 steps of 0.08 s, and the decision variables are the 10 steering angles. The rollout moves the rear axle and the errors are measured at the tracked point (the offset fix, CHANGELOG 3.13).

**Why SLSQP** (`scipy.optimize.minimize`, steering bounds, `maxiter` 6, `ftol` $10^{-3}$, warm start from the shifted last solution):

| Reason | Source |
|------------------------------------------|------------------|
| SciPy is available on the lab's target (Foxy, Python 3.8, SciPy 1.3.3), so nothing extra to install | `docs/COMPATIBILITY.md` |
| It handles the box bound on steering natively, so the bound is exact | Theory, NMPC section |
| The problem is small (10 variables) | code |
| It also handles the nonlinear inequality constraints that `NonlinearObstacleMpc` needs | `mpc_obstacles.py` |
| With 6 iterations and a warm start each tick refines the last answer, like a real-time iteration | Theory |

The MATLAB port has no Optimization Toolbox, so a projected Levenberg-Marquardt is used there (CHANGELOG 3.10). The repo has no benchmark of other solvers (CasADi, IPOPT and so on). The honest wording is "SLSQP was the dependency-free choice that handles bounds and constraints", not "SLSQP was the best".

Known limits: the code does not check `res.success`, so an unconverged step is used as is. There is no integral term, so a constant trim is not removed. Speed is not a decision variable.

### 4.5 Shared feed-forward

For a steady turn on a bicycle model, $\tan\delta = L\kappa$. If the error is zero, the steering must be $\delta_{ff} = \arctan(L\kappa)$. Adding it as feed-forward means the feedback only corrects errors and does not have to hold the turn.

| Controller | How it uses $\delta_{ff}$ |
|-----------|-------------------------------------------------|
| PID | Added to the feedback; curvature from the look-ahead point |
| LQR | Added to $Ke$; curvature from the look-ahead point |
| LTV-MPC | Weighted preview over five horizon steps |
| NMPC | Not a term; the rollout needs it implicitly; used as the warm-start value for the new last step |

`run_manager.shape()` stores $\delta_{ff}$ for every controller, so the telemetry compares the steering sent with what the geometry needs (`steer_ff`, `steering.png`, `peak_geometry_use`).

A related fix (CHANGELOG 3.13): the heading error was $\theta_{lookahead} - \theta$, which is non-zero on any curve even with perfect tracking and made every controller cut the inside. It is now $\theta_{nearest} - \arctan(a\kappa) - \theta$ with $a$ = `ref_offset`. In simulation the RMS fell from 5.6 to 9.7 cm to 0.6 to 1.8 cm (Gazebo, one lap, CHANGELOG section 4).

### 4.6 Steering limit 0.4084 rad

The LIMO driver (`limo_driver`) applies 28 deg to the inner wheel. With wheelbase 0.20 m and track 0.172 m, the rear-axle radius is

$$R = \frac{L}{\tan 28^\circ} + \frac{\text{track}}{2} = 0.462\ \mathrm{m},$$

so the centre angle is $\arctan(0.2/0.462) = 0.4084$ rad and the curvature limit 2.16 1/m. The controllers had used 28 deg as the centre angle (0.4887 rad, radius 0.376 m, curvature 2.66 1/m), so they planned steering the driver clamped (CHANGELOG 3.18). The fix is in the defaults of the four controllers, `run_laps.py`, `limo_test_suite.py` and `sim_limo.py`. The MATLAB `config_limo.m` still has 0.48869 on purpose, so saved MATLAB results do not change.

## 5. Safety and obstacles

### 5.1 Always on

| Safeguard | What it does |
|--------------------|----------------------------------------|
| KILL | A button in the live view publishes a latched `/run_kill`. The controller stops, ends the run as `KILLED` and refuses any path until RE-ARM and a new path. The suite cancels the rest |
| Zero command on lost mocap | No odometry for 0.5 s gives a zero Twist at every tick; without the guard it resumes when odometry returns |
| No run from a stale pose | A new path is accepted only if odometry is fresh |
| Zero commands on exit | 10 zero commands on Ctrl-C (one message can be lost on WiFi), and 10 more ticks after a run |
| Context alive on Ctrl-C | So the final zero can be published (`ros_compat.init_ros()`) |
| PoseGate | Drops a frame with a wrong marker distance or an impossible jump; accepts a new pose after 10 jumps in a row |
| Lost-marker rejection | Skips a frame with a missing marker, `cond <= 0`, or exactly (0, 0, 0) |
| Nothing listening | An error is logged if nothing subscribes to the command topic |
| Lab check | `hw_check` before tracking |

Why PoseGate: one ghost marker swung the heading, and so the steering, in the desk test with the glitch mode (`tools/hw_test/`). Its thresholds (3 cm, 1.5 m/s, 3 rad/s) are estimates, not measured on the real system (HANDOFF). A ghost that lasts longer than about 10 frames would be believed.

### 5.2 Optional guard, off by default

`guard:=true` turns on: slow down on drift, re-plan the way back, stop at a large error, stop on leaving the lab, on too many rejoins, on lost odometry for 3 s, and on an approach that takes too long (levels in system guide 3.3).

Why off: the comparison wants to measure how each controller recovers from its own errors. A guard that slows or re-plans would hide that. The owner confirmed this on 2026-11-06 (HANDOFF). Consequence: with the guard off there is no hard stop at the lab boundary. What stops the robot is KILL, the hardware stop, and the zero command on lost mocap. Keep the hardware stop in reach.

### 5.3 Obstacles as an add-on

Obstacles are virtual in the add-on (`obstacle_lap.launch.py`): the bridge (or `sim_limo`) publishes them on the latched `/hardware_obstacles`, the live view draws them and the controllers avoid them. Real 0.60 m cylinders can be put at the same places. Nothing outside `lab_obstacles.py` depends on the add-on.

### 5.4 Why a CBF filter

| Question | Answer |
|-------------------------|-----------------------------------|
| Why one filter for all four controllers? | One safety layer between any controller and `/cmd_vel`, so the safety behaviour is the same for each and the comparison stays fair |
| Why not only MPC constraints? | PID and LQR have no look-ahead and no optimiser. A CBF is closed-form geometry that works on any command |
| What does it change? | The minimum: the safe command closest to the wanted one; a safe command passes unchanged (`CLEAR`) |
| Why no solver? | A convex polygon clip in the $(v, \omega)$ plane: no iteration, no failure |

The barrier, the half-plane and the two layers are in the controllers guide, 6.1. An extra approach layer was needed because a distance barrier alone reacts too late for a robot that cannot turn on the spot. Before that change the old filter stopped 0.28 m from the obstacle (CHANGELOG 3.14). Result: the robot goes round and rejoins, and stops (`CBF_STOP`) only if there is no safe way forward. In Gazebo with a 0.15 m cylinder the minimum clearance was 0.300 to 0.303 m against the 0.30 m target, with no stops (CHANGELOG section 4).

### 5.5 Which side to pass on

The filter commits to a side once per obstacle, from which side of the path ahead (next 3 m) the obstacle centre lies. The heading is only a fallback. The heading-based choice failed in rehearsal: with the obstacle 0.15 m off the line and a 16 deg heading error, the robot went round the outside, to $y$ = -0.50 (the lab edge). The whole path cannot be used, because the approach plus the track prefix passes the same obstacle twice in opposite directions, so only the next 3 m are used.

### 5.6 Steering smoothing

Near an obstacle, and for 0.6 s after the filter last acted, the steering may change at most 2.0 rad/s, and a hysteresis of 0.08 rad stops the filter flipping between `CLEAR` and `CBF_ACTIVE` at the cone edge. The first version jumped from limit to limit (more than 11 deg in one tick, 13 to 24 times per lap). The real servo moves at about 1.5 to 2 rad/s, so it would jerk and wear. After the change there were 0 to 1 such jumps per lap in rehearsal, with the clearance unchanged at 0.59 to 0.61 m (CHANGELOG 3.21). The jump count was an ad hoc analysis; no report computes it (section 6).

### 5.7 Placement of the two obstacles

| Item | Value | Why |
|--------|--------------------------|--------------------------|
| Size | 0.60 m diameter | A real cylinder that is easy to find |
| Position | On each straight, 1 m past the middle in the driving direction | Away from the curves, so the obstacle effect is not mixed with a curve |
| Offset | Centre 0.15 m outside the track line | The robot must swerve inward, never towards the lab edge |
| Second obstacle | The first, mirrored through the track centre | One on each straight |
| Start | `start_at:=begin` | 3 m of straight before the first obstacle (user request: more run-up) |

The centres, the 0.60 m clearance and the error conditions are in the system guide, 3.7.

### 5.8 CBF mode or constraint mode (LTV-MPC and NMPC only)

`obstacle_mode:=constraint` replaces the CBF with `mpc_obstacles.py`; `obstacle_lap.launch.py compare_modes:=true` runs `mpc`, `nmpc`, `mpc:constraint`, `nmpc:constraint` as one session `obstacle_modes_<stamp>`. The solvers are described in the controllers guide, 6.2 and 6.3.

The CBF filter clamps to the edge of the collision cone, so the robot drives a straight tangent to the safety circle and then round it. Constraint mode plans over a 3 s look-ahead, so it should turn earlier and more smoothly (`docs/LAB_GUIDE.md`). This is an expectation: no hardware result exists, and the owner is still deciding whether constraint mode should become the default for the MPCs (HANDOFF).

Limits: the linear case adds `MODEL_MARGIN` 0.10 m to the keep-out because the linear model is wrong while turning hard. Constraint mode has no hard steering rate limit; only the cost penalises steering change.

## 6. Metrics and what each says

Per-tick data are in `telemetry.csv` (23 columns, system guide 4.2). The summary metrics are:

| Metric | Computed from | Summarised in |
|--------------|--------------------------|--------------------|
| RMS cross-track error per lap | Scored ticks of the lap (`stats_lap.py`) | `report.md`, `laps.png`, `summary.csv` |
| Largest error | Maximum of $\lvert e_{ct}\rvert$ in the lap | `report.md` |
| First lap against last lap | RMS of lap 1 and of the last lap | `report.md`, `laps.png` |
| Share at the steering limit | Ticks with $\lvert\delta_{sent}\rvert \geq 0.98\,\delta_{\max}$ over all ticks | `report.md` |
| Peak steering use | Largest $\lvert\delta_{sent}\rvert/\delta_{\max}$ | `report.md`; geometry value in `summary.csv` |
| Largest error per curve | Per curve ($\lvert\kappa\rvert > 0.1$), maximum error, averaged (`recovery()`) | `report.md`, `recovery.png` |
| Distance to settle | Driven distance after a curve until error stays below 2 cm for 0.5 m | `report.md`, `recovery.png` |
| Curve exits never settled | Count of exits that did not settle before the next curve | `report.md` |
| Approach time | `approach_time_s` | `report.md` |
| Rejoins | Guard rejoins (0 with the guard off) | `report.md` |
| Steering rate | RMS of the steering change per second | `summary.csv`, `summary.json` |
| Lap time | Seconds per lap | `summary.csv` |
| Solve time | `solve_ms` (LTV-MPC, NMPC; 0.0 for PID, LQR) | `telemetry.csv` only |
| Clearance to obstacles | `min_clearance`, per tick | `telemetry.csv` only |
| Steering jumps | Not computed anywhere | not summarised |

**What each tells about a controller.**

- *RMS and largest error.* RMS is the typical error and punishes sustained offsets. The largest error is the worst single point: a spike such as a mocap glitch or one bad curve entry shows there but hardly moves the RMS. Low RMS with high maximum means rare spikes (look at `error_along.png` at the curve entries); moderate RMS with moderate maximum means a steady offset. Report both.
- *First and last lap.* Settling and repeatability. With learning removed, a large change between laps means drift (trim, integral, battery, mocap), not "learning".
- *Share at the limit and peak use.* How hard the controller works. A high share means no reserve: it cannot correct further in that direction. It also reveals chatter on straights (LQR). Compare the peak with what the geometry needs (48 % at R1) to see the extra effort. Low error at 100 % steering use means no reserve.
- *Error per curve and settle distance.* The curve is the main disturbance on this track. They show how well and how fast the controller recovers. "Never" means it did not settle before the next curve.
- *Approach time.* Mostly set by the planner and `approach_speed`, not by the controller. Useful to check that the approach did what was planned.
- *Steering rate.* Smoothness, servo wear and jerk. A high rate means a controller near bang-bang.
- *Lap time.* The speed law is shared, so lap time mostly reflects steering use.
- *Solve time.* Real-time feasibility: the NMPC solve must fit in its 40 ms tick.
- *Clearance.* The safety margin. The CBF aims for 0.30 m from the cylinder edge.

Also recorded but not summarised: `v_actual`, `heading_err` and `safety_status` (`NO_OBSTACLES`, `CLEAR`, `CBF_ACTIVE`, `CBF_STOP`; `MPC_CLEAR`, `MPC_ACTIVE`, `MPC_STOP` in constraint mode).

**Caveats for quoting numbers.**

- The error is measured against the mocap pose, so the measurement noise floor is part of the number. On the real system the floor is unknown until the first run. Do not expect the simulation values of 0.2 cm on hardware.
- `report.md` shows RMS and maximum as the last-lap values and the overall maximum. Per-lap values are in `laps.png` and `summary.csv`.
- The last curve of a run is not counted in the recovery columns, because no curve follows it.

## 7. Tooling

**Interactive prompts.** `launch_prompts.py` (`ask_first`, `Ask`) is placed first in the entry launch files (`record_lap`, `compare_lap`, `obstacle_lap`, `r05_lap`, `hw_tracking`, `hw_check`, `limo_test`), so nobody needs to know the command-line arguments. A value given as `name:=value` is used and never asked; a missing value is asked; with no terminal the default is used. The rules are in the system guide, section 2.

**Logs.** `log_dir ''` means `<workspace>/evaluation`, found from the install folder, so it is the same place from any directory. The layout is in the system guide, 4.1. A run that never moved (refused start) is listed in the tables and left out of the charts.

**Sending the logs.** `tools/send_logs.sh` copies the log folders from the lab machine to the PC with one password prompt (system guide, 3.19). A real transfer has not been run (it needs the password). The receiver passed its own check.

**ROS distributions.**

| | Foxy | Humble |
|---------|------------------------|--------------------------|
| Python | 3.8 | 3.10 |
| Difference handled | `rclpy.init(signal_handler_options=...)` does not exist | The default Ctrl-C handler shuts the context down first, so the final zero command would fail |
| Fix | `ros_compat.init_ros()` | Uses `SignalHandlerOptions.NO` |

Foxy was checked only by compiling all files with Python 3.8, plus a fake `rclpy` with Foxy's API and Ubuntu 20.04's library versions (numpy 1.17.4, scipy 1.3.3, matplotlib 3.1.2). It has not run on a real Foxy machine. `tools/check_deps.py` and `tools/setup_ros2.sh` check and install the dependencies. Python 3.10 on Foxy is not supported, because the ROS libraries are built for 3.8.

**Rehearsal.** `tools/hw_test/` (fake mocap, stub messages, `run_hw_tests.py` with the cases normal, mirror, swap, nolisten, glitch) and `sim:=true` on every suite. `sim_limo` is deliberately imperfect (steering gain 0.89, bias 0.02 rad, lag 0.12 s, rate limit).

## 8. Limitations and open items

| # | Item | Detail |
|-----|--------------------|-----------------------------------|
| 1 | Not run on the real LIMO | The suites, approach, CBF and metrics were rehearsed with `sim_limo` only. The topic `/limo1/cmd_vel` comes from the lab robot's topic list (CHANGELOG 3.12) and the owner reports it was tested on hardware. HANDOFF and LAB_GUIDE still say no tracking suite has run; update them after the first lab day |
| 2 | Sim stand-in | `sim_limo` is a kinematic bicycle with some imperfection: no tyre slip, battery sag or deadband. The controller ranking in sim may not hold on the floor |
| 3 | Real `phasespace_msgs` | The package is not on the development machine. The bridge was tested against a stub with the fields `x, y, z, cond` and against a build of the `mocap-ros2` branch for some cases. If the real field names differ, `marker_cb` must change. Never source the stub where the real package exists |
| 4 | PoseGate thresholds | Estimates. Raise `max_baseline_error` if good tracking gives dropped-frame warnings |
| 5 | `ref_offset` 0.10 m | An estimate. A 5 cm error left 1.6 to 5.1 cm RMS in a fake-robot test, depending on the controller. Measure it on the robot (`hw_check drive:=true`) |
| 6 | Steering trim, left and right | LQR and NMPC have no integral term, so a constant trim stays as an offset. `hw_check` shows left and right differences |
| 7 | Constraint mode steering rate | Not rate-limited; only the cost discourages fast changes |
| 8 | Guard off by default | No hard stop at the lab boundary |
| 9 | Robot does not stop on its own | If commands stop (WiFi drop), the LIMO may keep driving. `hw_check` tests this. A watchdog on the robot would fix it and is not built |
| 10 | Zero Twist in the driver | `limo_driver.cpp` computes $r = v/\omega$, so a zero command is 0/0 and the wheels may twitch on stop. Not our code |
| 11 | Motor deadband | The CBF can command about 0.03 m/s near obstacles. If the motors stall at that speed, the robot sits still. `hw_check` does not test this yet |
| 12 | LTV-MPC is unconstrained | Steering limit by clipping; two empirical factors in the model (4.3) |
| 13 | NMPC convergence | 6 iterations, `res.success` is not checked |
| 14 | Metrics gaps | No summary of solve time, clearance or steering jumps |
| 15 | No clothoids | Curvature steps from 0 to $1/R$ at each joint |
| 16 | Open paths | Laps only for closed paths; an open drawn path is driven once |
| 17 | Approach planned once | With the guard off, no re-plan if the robot drifts during the approach |
| 18 | Lap learning | Removed on purpose; design in `docs/FUTURE_lap_learning.md`. Not to be added back without asking the owner |
| 19 | Foxy not run | Python 3.8 syntax and API checks only |
| 20 | Licence | `package.xml` says Apache-2.0, `LICENSE` has MIT text. Owner to choose |

Next steps (HANDOFF): run `hw_check.launch.py` on the real robot, then `hw_check.launch.py drive:=true ref_offset:=<measured>`, act on `report.json`, then a single controller at low speed (`record_lap`), then `compare_lap`.

## 9. Likely questions on presentation day

### Test design

**Why a fixed track and not a hand-drawn path?**
It is repeatable and checkable: the path and its curvature are known exactly. A drawn path differs every time, and how hard it is depends on how tight the user drew relative to the steering limit.

**Why $R$ = 1.0 m?**
The steering limit allows a turn of about 0.46 m. $R$ = 1.0 m needs 48 % of the limit, so every controller can do it and the differences are about quality. In the sim study the R1 error was 0.2 to 0.8 cm. $R$ = 0.4 m is beyond the limit and gave about 10 cm for all four.

**Why this size and place, centre (3.25, 1.25)?**
Two 4 m straights and $R$ = 1.0 m give a track of x 0.25 to 6.25 m and y 0.25 to 2.25 m inside the mocap area x -0.5 to 7.0 m and y -0.5 to 3.0 m. Every edge has 0.75 m margin for drift and for swerving round obstacles.

**Why start in the middle of a straight?**
The robot starts at zero curvature and has 2 m to reach speed before the first curve. The lap boundary is also on a straight.

**How is the start side chosen?**
Top, unless the robot is below $y$ = 1.25 m, then bottom. It is the owner's rule, chosen right before each run from `/hardware_odom`.

**Why CCW?**
It is the default convention (left turns). CW is available, and a CW run also checks for left-right asymmetry in the real steering.

**Why three laps?**
Lap 1 includes settling after the approach (the first metre is not scored). Laps 2 and 3 show whether the result repeats, and the first and last lap are shown side by side.

**Why run the controllers one after another with a buffer?**
Same start, same track, same conditions. Each controller is a fresh process. The robot is held at zero for 10 s between runs, then drives itself to the start.

**What is the R0.5 add-on for?**
It needs 93 % of the steering, so it shows which controller copes at the edge of the robot's ability.

### Getting to the start

**Why does the robot not U-turn in place?**
It is an Ackermann car. It cannot turn on the spot, and the planner does not reverse. It follows a Dubins curve with at least 0.6 m radius.

**Why a 0.6 m approach radius when the limit is 0.46 m?**
The curve needs 79 % of the limit, which leaves margin for servo error.

**Why a fixed merge rule and not the shortest or best-scored path?**
It is predictable and explainable, and the same input always gives the same plan. The shortest Dubins path to the start pose looped near edges (up to 1.3 m outside the lab). Scored rules were hard to predict, and one rode up to 12 m round the track.

**What is the rule in one sentence?**
Find the track point in front of you going your way, merge 1 m further on, follow the track to the start; no loops, stay inside the lab, clear of obstacles; if that fails, move the merge point on, and as a last resort drive straight to the start.

**What happens if the robot is next to an edge, facing out?**
It may go up to 0.3 m past the edge, with a warning. If that is not enough, it does not move and the run ends `ABORTED`, because outside the mocap area the robot is lost.

**How well does the rule work?**
Over 840 start poses: 568 merge, 37 merge onto the start, 187 go straight to the start, 48 refused. The worst excursion was 0.27 m outside the lab (old rule: 1.30 m).

### Controllers

**Why these four?**
A simple baseline (PID), an optimal linear controller (LQR), a controller that looks ahead on a changing model (LTV-MPC), and one that uses the full nonlinear model (NMPC).

**Which solver for the NMPC and why?**
SciPy SLSQP: available on Foxy and Python 3.8 with nothing extra to install, handles the steering bounds natively, and handles nonlinear constraints for the obstacle mode. Ten variables, 6 iterations, warm start. Other solvers were not benchmarked.

**How is the LQR solved?**
With the continuous algebraic Riccati equation at the current speed and curvature. The result is cached, so repeated operating points are not recomputed.

**Is the LTV-MPC a real MPC?**
It is a finite-horizon optimal controller solved in closed form by a Riccati recursion, with no constraints inside. The steering limit is applied by clipping. In obstacle constraint mode a constrained solver is used.

**Why the feed-forward $\arctan(L\kappa)$?**
On a bicycle model a steady turn needs exactly that steering. With it, the feedback only corrects errors instead of holding the turn.

**Why 0.4084 rad and not 28 deg?**
The driver applies 28 deg to the inner wheel. The centre angle at that setting is 0.4084 rad (radius 0.46 m). Using 28 deg as the centre angle made the controllers plan steering that the driver clamped.

**Why do all four share the speed law?**
So that the comparison isolates the steering law, not the speed.

### Safety and obstacles

**What happens if mocap drops out?**
The controller sends a zero command whenever odometry is older than 0.5 s, and a new run is not started from a stale pose. If the data comes back, a run goes on (guard off). The bridge also drops frames from ghost markers.

**What stops the robot?**
KILL in the live view, the hardware stop, and the zero command on lost mocap. The optional guard (off by default) adds slow-down, re-plan and stop on drift.

**Why is the guard off?**
To measure how each controller recovers on its own. A guard would hide it.

**Why a CBF and not only MPC constraints?**
One safety layer for all four controllers, including PID and LQR that cannot have constraints. It changes the command as little as possible and is closed form. For the two MPCs a constraint mode was also built and is compared.

**How does the CBF decide which side to pass?**
From which side of the path ahead (next 3 m) the obstacle centre lies. The heading alone failed in rehearsal when it was 16 deg off.

**Why smooth the steering near obstacles?**
The filter jumped from limit to limit, 13 to 24 times per lap, while the real servo manages about 1.5 to 2 rad/s. The 2 rad/s rate limit brought it to 0 to 1 jumps per lap in rehearsal.

**Why put the obstacle centres 0.15 m outside the line?**
So that the robot is pushed inward, into the track, and never towards the lab edge. It passes about 0.45 m inside the line.

**What do you expect from CBF against constraint mode?**
Constraint mode looks 3 s ahead, so it should turn earlier and more smoothly. The CBF clamps at the cone edge, so it drives a tangent. This is an expectation; there is no result yet. Constraint mode has no hard steering rate limit.

### Metrics

**What does the RMS cross-track error tell you that the maximum does not?**
The typical error over the whole lap, including sustained offsets. The maximum is one worst point. Low RMS with high maximum means rare spikes; both are needed.

**Why report the share of time at the steering limit?**
Error alone hides effort. A controller with a small error that sits at the limit for 14 % of the lap (LQR in the R1 study) has no reserve to correct more.

**Why look at the error after a curve?**
The curve is the main disturbance. The error per curve and the distance to settle show the recovery. "Never settled" means it did not recover before the next curve.

**Why compare the first and the last lap?**
Settling and repeatability. With learning removed, a big difference means drift.

**Is real-time feasibility measured?**
`solve_ms` is recorded in the telemetry for the two MPCs but not yet summarised in the report. The NMPC runs at 25 Hz, so the solve must fit in 40 ms.

### Honesty questions

**Has this run on the real robot?**
Not for these suites. All numbers are simulation. The first lab day starts with `hw_check`.

**Do the simulation results transfer?**
The shape should (limit effects, ranking at R0.4), the absolute numbers will not. The sim stand-in has no slip, deadband or battery effects, and the mocap noise floor is unknown.

**What would you do next?**
Run `hw_check`, measure `ref_offset`, run one controller at low speed, then the comparison, then the obstacle add-on, and summarise solve time and clearance in the report.

## Appendix A. Documents that disagree with the code

The code wins. None of these changes behaviour.

| # | Document says | Code says |
|----|----------------------------|----------------------------|
| 1 | `CbfFilter.__init__` default `max_steer` 0.48869 | The nodes pass 0.4084, so the default has no effect |
