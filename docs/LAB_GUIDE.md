# Lab guide: one lab day, step by step

Commands on `/limo1/cmd_vel` and hand-drawn path tracking were tested on the real LIMO (2026-11-06). The lap suites, the obstacle add-on and the way to the start were rehearsed with the stand-in robot (`sim:=true`) only. Follow the steps in order. Stop at the first step that fails.

## The lab and the track

Mocap frame: standard ROS, +x forward, +y left. Marker 0 is the front, marker 1 the rear. Heading points from 1 to 0. The pose is the marker midpoint.

Mocap area used: x -0.5..7.0, y -0.5..3.0 (`LAB_X`, `LAB_Y` in `track_tools.py`, the default everywhere). Track: radius 1.0 m, straights 4.0 m, centre (3.25, 1.25). It spans x 0.25..6.25, y 0.25..2.25, so every edge has 0.75 m margin.

![The lab area, the R1 lab track, the start points and the obstacle spots. +x forward, +y left (ROS frame).](figures/lab_track.png)

The obstacle spots are in the table below (orange in the figure for CCW runs, grey for CW). Start points (middle of a straight): bottom (3.25, 0.25), top (3.25, 2.25). Default `start_side:=auto`: the top straight, unless the robot stands below the centre line (y < 1.25), then the bottom one. It is chosen right before each run from `/hardware_odom` and logged as `Start: middle of the top straight`. `start_side:=top|bottom` forces a side. In compare_lap every lap ends at the start, so all controllers start from the same side. Default `direction:=ccw` (left turns). General argument `start_at:=middle|begin` (default `middle`) on run_laps, record_lap, compare_lap and hw_tracking: `begin` starts where the straight begins, at the end of the curve.

| Direction | Obstacle 1 | Obstacle 2 |
|---|---|---|
| ccw | (4.25, 0.10) | (2.25, 2.40) |
| cw | (2.25, 0.10) | (4.25, 2.40) |

## 0. One-time setup

On the lab machine:
```bash
cd <path to>/HW_Ackermann_Drive
git pull
```
```bash
colcon build --symlink-install --packages-select limo_mocap_suite
```
```bash
source install/setup.bash
```
Source the PhaseSpace workspace too (`phasespace_bringup`, with `phasespace_msgs`), and use the robot's domain:
```bash
export ROS_DOMAIN_ID=<same as the robot>
```
On the machine that receives the logs:
```bash
tools/send_logs.sh --check-receiver
```
It checks openssh-server, that sshd runs, port 22, ufw and password login, and prints this machine's IPs. Write the IP down. Fixes are in the troubleshooting table.

## 1. Check the frame and the markers

Robot still:
```bash
ros2 launch limo_mocap_suite hw_check.launch.py
```
Then the slow test drive (0.15 m/s, about 30 s, keep 3 m clear):
```bash
ros2 launch limo_mocap_suite hw_check.launch.py drive:=true ref_offset:=<your value>
```
It checks mocap rate, that something listens on the command topic, that forward is forward, that yaw points along the driving direction (markers swapped?), and that a left command turns left (frame mirrored?). Details are in [README.md](README.md#first-the-lab-check). Any FAIL: fix it before going on. Report goes to `evaluation/hw_check_<stamp>/report.json`.

## 2. Optional rehearsal, no robot

Every suite takes `sim:=true`. It uses the stand-in robot and needs no lab:
```bash
ros2 launch limo_mocap_suite record_lap.launch.py controller:=lqr sim:=true
```

## How every run goes

- The robot may start anywhere in the mocap area. On a closed track it joins the track like a car joining a road, always by the same rule: it finds the closest part of the track that is in front of it and runs its way, merges 1 m further along with one smooth curve (no loops, 0.5 m from obstacle edges), then follows the track to the start. If the start comes first, it merges straight onto the start. If it stands sideways near an end of the loop and no smooth merge exists, it takes the shortest curve straight to the start (the log says "no smooth merge"). Examples below; `tools/planner_check.py` redraws them.

![The way to the start from six start poses: arrow = robot, star = start, orange = merge curve, blue = ride along the track. Last panel: obstacle run.](figures/fixed_merge_rule.png)
 The whole approach is phase `APPROACH` (`approach_radius` 0.6 m, `approach_speed` 0.25 m/s), logged apart from the laps (`TRACK`). The terminal shows one line before the robot moves, for example: `Way to the start: join the path at (5.21, 2.25) by a 1.36 m curve (RSR, turning 120 deg), then 3.18 m along the path to the start`. Read it during the 5 s `start_delay`.
- Where to put the robot: at least 0.6 m inside the mocap area edges, and not facing straight out. Anywhere else is fine. A car that cannot reverse, standing at an edge and facing out, cannot turn without going further out: the way to the start may go up to `approach_spill` (0.3 m) past the edge, with a warning. If even that is not enough, the robot does not move and the run ends `ABORTED` with "cannot reach the start ...": move or turn the robot and start again. Raise `approach_spill:=0.5` only if the mocap really sees that far.
- Point 0 of the path is the start. `start_side:=auto|top|bottom` (auto: top, or bottom if the robot is below y = 1.25), `direction:=ccw|cw`.
- A 5 s warning (`start_delay`) comes before the robot moves.
- Common arguments: `v_nominal` (0.30 m/s), `laps`, `start_side`, `direction`, `robot_ns` (`/limo1/`).
- **KILL** in the live view stops the robot and cancels the rest of the suite.
- `guard` is off by default. There is no hard stop at the lab boundary. What stops the robot: KILL, the hardware stop, and a zero command when mocap is lost. Keep the hardware stop in reach.
- Logs go to `<workspace>/evaluation`, found from the install folder, wherever you launch. Each test folder holds `settings.json`, `results.json`, `report.md`, comparison charts and `R1/<run>/` with `telemetry.csv`, `summary.json` and charts.

## 3. Suite 1: record one controller

```bash
ros2 launch limo_mocap_suite record_lap.launch.py controller:=lqr
```
`controller` is `pid`, `lqr`, `mpc` or `nmpc`. Three laps. Log: `evaluation/record_<controller>_<stamp>/`.

## 4. Suite 2: compare all four

```bash
ros2 launch limo_mocap_suite compare_lap.launch.py
```
PID, LQR, LTV-MPC and NMPC in turn, three laps each. Between controllers the robot is held stopped for `buffer_time` (default 10 s), then the next one drives itself back to the start. Log: `evaluation/compare_<stamp>/`, with one `report.md` comparing them.

## 5. Obstacle add-on

```bash
ros2 launch limo_mocap_suite obstacle_lap.launch.py
```
Options: `controllers:=...`, `obstacle_mode:=cbf|constraint`, `direction`. Two virtual cylinders, 0.60 m diameter, at the spots in the table above. Each centre is 0.15 m outside the line and 1 m past the middle of a straight in the driving direction, so the robot is pushed inward. This add-on starts at the beginning of the straight (`start_at:=begin`), so the robot has 3 m of straight before the first obstacle: ccw from (1.25, 0.25) heading +x, cw from (5.25, 0.25) heading -x. The CBF keeps the tracked point 0.60 m from each centre, about 0.45 m inside the line. Near an obstacle the CBF limits how fast the steering command may change (`cbf_steer_rate` 2.0 rad/s), as the real servo is slow. They show as discs in the live view. You may put real 0.60 m cylinders at the same spots. Log: `evaluation/obstacle_lap_<stamp>/`.

CBF is the default for every controller. To compare LTV-MPC and NMPC with the CBF against obstacles as constraints:

```bash
ros2 launch limo_mocap_suite obstacle_lap.launch.py compare_modes:=true
```
It runs mpc, nmpc (CBF filter), then mpc:constraint, nmpc:constraint, in one session `evaluation/obstacle_modes_<stamp>/` with one report. Labels: "LTV-MPC" and "LTV-MPC constraints". In the `controllers` list of run_laps an entry such as `mpc:constraint` does the same for one controller. The CBF filters the command after the controller: it clamps the heading to the edge of the collision cone, so the robot drives a straight tangent to the 0.6 m safety circle, then round it. Constraint mode plans round the obstacle over the look-ahead, so it turns earlier and more smoothly. PID and LQR have no look-ahead, so they use the CBF only.

This add-on is separate: nothing else depends on `lab_obstacles.py`.

## Changing things in the lab

**Change a controller setting without editing code.** Every lap suite and `hw_tracking` take `controller_params`: name:=value pairs, in quotes, passed to the controller node. The values are logged in the run's `settings.json`.

```bash
ros2 launch limo_mocap_suite record_lap.launch.py controller:=lqr 'controller_params:=q_cte:=300 r_steer:=0.8'
```

Useful names: PID `kp_cte ki_cte kd_cte kp_heading lookahead_pts`; LQR `q_cte q_heading r_steer gain_cache`; LTV-MPC and NMPC `q_cte q_heading rd_steer horizon_steps`; all `v_nominal max_steer ref_offset`. `gain_cache:=false` makes the LQR solve the Riccati equation every tick (same gains, for checking). Whole numbers are turned into decimals where the node expects one.

**Edit the code.** Build once with `--symlink-install`. After that, an edited `.py` or launch file takes effect at the next `ros2 launch`, with no rebuild. A new file or a change to `setup.py` still needs `colcon build`.

```bash
colcon build --symlink-install --packages-select limo_mocap_suite
```

To comment out a line, put `#` in front of it. Keep track of what you changed:

```bash
git diff > ~/lab_changes_$(date +%Y%m%d).patch
```

Before the next `git pull`, put your edits aside with `git stash`, and bring them back with `git stash pop`. To get them to the main machine, send the patch file along with the logs, or copy it with `scp`.

## 6. Send the logs

On the lab machine, from the repo:
```bash
tools/send_logs.sh
```
It lists the folders in `evaluation/`, newest first. Press Enter for all not yet sent, `a` for all, or type numbers. Then it asks for the IP of the receiving machine. The user is `alienator`. The ssh password is asked once. Files land in `/media/alienator/Storage/Jay/Robotics/ALNC/HW_Ackermann_Drive/evaluation/from_lab/`. Sent folders are remembered in `evaluation/.sent_log`.

Flags: `-i IP`, `-u user`, `-p path`, `-d dir`, `--dry-run`. Try the dry run first:
```bash
tools/send_logs.sh --dry-run
```
Both machines must be on the same network.

## 7. Troubleshooting

| Problem | Cause and fix |
|---|---|
| Robot does not move, hw_check says nothing is subscribed to the command topic | Robot off, `ROS_DOMAIN_ID` differs from the robot's, or wrong `robot_ns`. Check `ros2 topic list` for `/limo1/cmd_vel`; use `robot_ns:=limo2` or `none` |
| `Track does not fit the lab` | The track or `start_side` leaves `LAB_X`/`LAB_Y`. Use the defaults, or change `lab_x`/`lab_y` to the real area |
| Markers not seen, no `/hardware_odom` | PhaseSpace workspace not sourced, mocap server off, markers hidden. hw_check shows rate and gaps |
| Robot drives away or turns the wrong way | KILL. hw_check: yaw about 180 degrees means swap `front_id` and `rear_id`; a left command turning right means a mirrored frame |
| Robot stopped after KILL and will not start | KILL holds zero commands until you press RE-ARM in the live view. A path sent while killed is refused. After RE-ARM, start the suite again |
| Robot keeps driving when WiFi drops | hw_check warns if it does not stop alone. Use the hardware stop |
| `ssh` or `scp` not found (lab machine) | `sudo apt install openssh-client` |
| Connection refused (receiving machine) | `sudo apt install openssh-server`, then `sudo systemctl enable --now ssh` |
| Connection times out | `sudo ufw allow ssh` if ufw is on. Check both machines are on the same network |
| Old radius study | `run_laps.launch.py` still takes `track_radii`, and the smaller radii (0.5, 0.4) fit inside the R1 footprint; `r05_lap.launch.py` is the ready-made R0.5 test |
