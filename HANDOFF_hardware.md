# Handoff: LIMO hardware work (2026-11-06)

Start here to continue the hardware work. `docs/README.md` has the commands, and `CHANGELOG.md` section 3.18 has the latest changes.

## Rules from the user
- **This folder is hardware only.** Sim and demo work belongs to another session, in `../Ackerman_Drive`. Don't add it here. `HANDOFF_demo_layout.md` is that session's note.
- **Don't commit or push until the user says so.** Stay on the current branch, `foxy-python38-support`, and don't create new ones.
- **Stops stay optional.** With `guard:=false` (the default), a drifting robot isn't stopped, even at the lab boundary, so recovery can be compared. The user confirmed this on 2026-11-06. Only KILL, the hardware stop and the zero command on lost mocap apply. Don't add a hard boundary without asking.
- **Don't add lap-to-lap learning back.** It was tried and removed. `docs/FUTURE_lap_learning.md` has the design and code.
- **Must work on Foxy (Python 3.8) and Humble.**

## Where it stands
The last commit is `bd97ea9`, which is local and not pushed: run manager, capsule tracks, lap test, MPC constraints.

**Uncommitted, and finished as far as a desk can test:**

| Change | Files |
|---|---|
| Lab check before tracking (new) | `limo_mocap_suite/hw_check.py`, `launch/hw_check.launch.py`, `setup.py` |
| Bridge drops ghost-marker frames (`PoseGate`), and speed is filtered | `phasespace_bridge.py` |
| Zero commands for 0.2 s on Ctrl-C; no run from a stale pose; error if nothing is listening; lab limits correct with `rezero_odom` | `run_manager.py` |
| `robot_ns` now works in the test suite (was hard-coded to `/limo1/cmd_vel`) | `limo_test_suite.py`, `launch/limo_test.launch.py` |
| `track_radius` refuses `rezero_odom:=true` | `launch/hw_tracking.launch.py` |
| Rehearsal with no robot or mocap (new) | `tools/hw_test/` |
| Docs | `docs/README.md` (tracked); `CHANGELOG.md` and `docs/Code_Explanation.tex` and its PDF (gitignored, so local only) |

Also untracked, left that way on purpose: `evaluation/` (13 MB of results).

## Update 2026-11-06 (afternoon): lab track, suites, obstacle add-on, log transfer
Asked by the user for the final project. Step-by-step lab instructions: `docs/LAB_GUIDE.md`; details: `CHANGELOG.md` 3.21.

| Change | Files |
|---|---|
| Lab area x -0.5..7.0, y -0.5..3.0 and the R1 lab track (4 m straights, centre (3.25, 1.25)) as defaults everywhere | `track_tools.py` (`LAB_X`, `LAB_Y`, `TRACK_*`), `plot_helpers.py`, `run_manager.py`, `track_publisher.py`, launch files |
| Race start mid bottom/top straight, `start_side` (default `auto`: top, or bottom if the robot is below y = 1.25; `pick_side()`), `direction` (ccw default) | `track_tools.capsule()`, `track_tools.pick_side()` |
| `obstacle_lap.launch.py compare_modes:=true`: mpc, nmpc (CBF) then mpc:constraint, nmpc:constraint, session `obstacle_modes_<stamp>`; `controllers` accepts `mpc:constraint` | `obstacle_lap.launch.py`, `run_laps.py` |
| Suite 1 `record_lap.launch.py controller:=X` (3 laps); suite 2 `compare_lap.launch.py` (all four, `buffer_time` 10 s held at zero) | new launch files, `run_laps.py`, `run_laps.launch.py` |
| Logs default to `<workspace>/evaluation/<session>_<stamp>` whatever the current folder | `run_laps.log_root()` |
| run_laps exit crash ("terminate called without an active exception") fixed: executor shut down and joined | `run_laps.py` |
| Obstacle add-on `obstacle_lap.launch.py`: two virtual 0.60 m cylinders 1 m past mid-straight, 0.15 m outside the line | `lab_obstacles.py`, `sim_limo.py` publishes them in rehearsal |
| CBF now picks the swerve side from which side of the PATH the obstacle lies (heading-based choice went round the outside with 16 deg heading error) | `obstacle_cbf.py` |
| `tools/send_logs.sh`: scp logs to user `alienator`, asks IP; `--check-receiver`. `.gitignore` now lets `tools/*.sh` through | `tools/send_logs.sh`, `.gitignore` |

| Later the same day, after user feedback: way to the start joins the track in the driving direction and follows it (fixed merge rule agreed with the user, no scores: closest track point in front going the robot's way, merge 1 m on, no loops, then follow the track; fallback straight to the start; obstacle_margin 0.5 m), keeps 0.25 m inside the edges, at most `approach_spill` 0.3 m out, else refuses (robot not moved); CBF steering rate-limited near obstacles (`cbf_steer_rate` 2 rad/s, `cbf_hold`, `CONE_HYST`); obstacle runs start where the straight begins (`start_at:=begin`); report_lap survives empty runs | `approach_planner.py`, `run_manager.py`, `obstacle_cbf.py`, `track_tools.py`, `report_lap.py`, launch files |

Open question to the user: make `obstacle_mode:=constraint` the add-on default for LTV-MPC/NMPC (smoother than the CBF filter).

| Docs and prompts (evening): launch prompts, `run_laps` rename, R0.5 add-on, comments on every function, four new PDFs + updated Theory (`docs/build_docs.sh`, Fira fonts), `tools/planner_check.py` | `launch_prompts.py`, `launch/*`, `docs/*`, `tools/planner_check.py` |

Note: the PDFs and `docs/tex/*.tex` stay local (user decision, 2026-11-06); `.gitignore` already excludes `*.tex` and `*.pdf`. Rebuild with `docs/build_docs.sh`.

Rehearsed with `sim_limo` only. Receiver (this machine) passed `--check-receiver`; a real transfer was not run (needs the password).

## What has been tested
`python3 tools/hw_test/run_hw_tests.py` passed on the final code on 2026-11-06, on Humble. It uses the stand-in robot, fake markers and the real bridge, `hw_check` and PID controller.

| Case | Result |
|---|---|
| normal | "OK to run tracking", with 4 warnings, all from the stand-in's imperfect steering |
| mirror | FAIL on the first left command |
| swap | FAIL on the straight, drove 179° from yaw |
| nolisten | Refused to drive |
| glitch | PID lap finished, largest error 1.7 cm, 66 bad frames dropped. rc 124 is expected: the controller stays up after a run |

**Not tested:**
- **Real LIMO and real mocap:** the user confirmed `/limo1/cmd_vel` and hand-drawn path tracking on the robot (2026-11-06). The lap suites, obstacle add-on and way-to-start rule have only run in simulation.
- **Real `phasespace_msgs`:** the package isn't on this machine. `tools/hw_test/src/phasespace_msgs` is a stub with only the fields the bridge reads. Never source it on a machine that has the real one.
- **Foxy:** only checked that the code parses as Python 3.8.
- **Dropped-frame limits:** 3 cm marker distance and the jump limits in `phasespace_bridge.py` are estimates, not measured.

## Next steps
1. **The user runs the lab check on the real robot.** Rebuild first, because `hw_check` is a new program. This is the blocking step:
   ```bash
   ros2 launch limo_mocap_suite hw_check.launch.py
   ```
   ```bash
   ros2 launch limo_mocap_suite hw_check.launch.py drive:=true ref_offset:=<measured value>
   ```
2. **Act on `evaluation/hw_check_<stamp>/report.json`.** The real numbers decide the next work:
   - **Mirrored frame or swapped markers:** fix first; tracking would run away.
   - **Left and right differ, or the radius isn't 0.46 m:** correct the limits and model.
   - **Trim:** LQR and NMPC have no integral, so they will hold an offset.
   - **Robot doesn't stop when commands stop:** a WiFi drop leaves it driving. Consider a small watchdog node on the robot.
   - **Bridge warns about dropped frames on good tracking:** raise `max_baseline_error`.
3. **Then tracking:** `hw_tracking.launch.py` with one controller at low speed, then `run_laps.launch.py` for all controllers on radii 1.0, 0.5 and 0.4 m.

## Worth checking on the robot
- **Motor deadband:** the CBF can command about 0.03 m/s near obstacles. If the motors stall that low, the robot sits still with no error. `hw_check` doesn't test this yet.
- **Zero Twist in the driver:** `limo_driver.cpp` computes `r = linear.x / angular.z`, so a zero command is 0/0, giving a NaN steering angle cast to an integer. Speed is zero, so it's probably harmless, but the wheels may twitch on stop. The driver is on the robot and isn't ours to change here.
- **Steering scale:** in the same driver, both turn directions use `right_angle_scale_`. Both scales are 2.47 in the copy in `../Ackerman_Drive`, but the robot's own copy may differ. The left vs right check in `hw_check` would show it.

## Good to know
- **Gazebo isn't used here.** Rehearse with `tools/hw_test/` or `run_laps.launch.py sim:=true`.
- **Thresholds:** `hw_check.py` has them at the top of the file. Its drive needs about 2.7 m of room, so `max_range` is 3 m.
- **Stopping processes:** kill test processes by id, never with `pkill -f`, because the other session may be running its own.
- **Machine load:** the other session's Gazebo runs use most of the memory and CPU, so check before starting anything heavy. Tctl read 94–98 °C even at low load, so that reading may not be accurate.
