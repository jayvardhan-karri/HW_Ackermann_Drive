# Dependencies and ROS 2 Foxy / Python 3.8 Compatibility

**Short answer:** the package runs on **ROS 2 Foxy with the system Python 3.8**, and on Humble with Python 3.10, with no code changes between them. **Do not install Python 3.10 for Foxy**: Foxy's ROS libraries only work with Python 3.8 (details in section 4).

On the target machine:

```bash
source /opt/ros/foxy/setup.bash
python3 tools/check_deps.py          # report only, installs nothing
./tools/setup_ros2.sh                # shows the apt packages it will install, asks, installs, builds, re-checks
```

---

## 1. Everything the code depends on

### ROS 2 packages

| Python module | apt package (Foxy) | Used by |
|---|---|---|
| `rclpy` (+ `rclpy.qos`) | `ros-foxy-rclpy` | every node; latched QoS for `/hardware_obstacles` |
| `nav_msgs.msg` (Odometry, Path) | `ros-foxy-nav-msgs` | bridge, controllers, GUIs |
| `geometry_msgs.msg` (Twist, PoseStamped) | `ros-foxy-geometry-msgs` | controllers, builder, test suite |
| `sensor_msgs.msg` (LaserScan, Imu) | `ros-foxy-sensor-msgs` | bridge, controllers |
| `std_msgs.msg` (Float32MultiArray) | `ros-foxy-std-msgs` | bridge, builder, live plotter |
| `phasespace_msgs.msg` (Markers, Marker) | **the lab's own package**; build it in `src/` or source its workspace | `phasespace_bridge` only |
| `launch`, `launch_ros` | `ros-foxy-launch`, `ros-foxy-launch-ros`, `ros-foxy-ros2launch` | `hw_tracking.launch.py` |
| `ament_index_python` | `ros-foxy-ament-index-python` | `ros2 run` / launch |

### Third-party Python

| Module | apt package | Tested on Foxy's version | Used by |
|---|---|---|---|
| `numpy` | `python3-numpy` | 1.17.4 | everything |
| `scipy.linalg.solve_continuous_are` | `python3-scipy` | 1.3.3 | LQR |
| `scipy.optimize.minimize` (SLSQP) | `python3-scipy` | 1.3.3 | NMPC |
| `scipy.interpolate.splprep/splev` | `python3-scipy` | 1.3.3 | builder **Smooth Path** |
| `matplotlib` (+ TkAgg backend) | `python3-matplotlib` | 3.1.2 | builder, live plotter, test suite |
| `tkinter` | `python3-tk` | 8.6 | GUI windows, builder screen size |
| `yaml` | `python3-yaml` | 5.3.1 | launch file (`obstacles:=`, `lab_x:=`) |
| `setuptools` | `python3-setuptools` | — | `colcon build` |
| standard library: `math`, `os`, `csv`, `time`, `threading`, `signal`, `glob` | — | — | — |

### Tools

`colcon` (`python3-colcon-common-extensions`) and `ros2` (comes with ROS). A display (`DISPLAY`) is needed for the three GUIs; the nodes themselves run headless.

### MATLAB folder

The MATLAB folder doesn't use the system Python. It needs base MATLAB, optionally the Control System Toolbox, and the ROS Toolbox for `run_on_limo.m` (see `matlab/README.md`). MATLAB's ROS Toolbox ships its own ROS 2 (Humble-based). Talking to Foxy nodes over DDS normally works for these standard message types, but it is not officially supported and was not tested against a Foxy system.

---

## 2. What was not Foxy-compatible, and the fix

The code was checked against the **Foxy branches of `rclpy`, `launch` and `launch_ros`** (source on GitHub) and against Python 3.8.

| Item | Humble | Foxy | Fix |
|---|---|---|---|
| `rclpy.init(signal_handler_options=...)`, `rclpy.signals.SignalHandlerOptions` | exist | **do not exist**: `TypeError` / `ImportError` at start-up | New `limo_mocap_suite/ros_compat.py` → `init_ros(args)`: uses the option on Humble, plain `rclpy.init` on Foxy. Used by the 4 controllers and the 3 GUIs. |
| Default Ctrl-C behaviour | shuts the ROS context down first (the final zero `Twist` would fail) | only raises `KeyboardInterrupt`; the context stays alive | That is why plain `rclpy.init` is correct on Foxy: the final zero `/cmd_vel` is still sent |
| `rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL` | yes | yes | — |
| `rclpy.executors.ExternalShutdownException` | yes | yes (code catches it generically anyway) | — |
| `launch_ros.actions.Node(executable=...)`, `launch.actions.OpaqueFunction` | yes | yes | — |
| `Node.declare_parameter(name, list_default)` | yes | yes | — |
| Python syntax | 3.10 | 3.8 | all 14 package files compile with 3.8 (no `match`, `X \| Y` types, `removeprefix`, …) |
| numpy / scipy / matplotlib APIs | new | 1.17 / 1.3 / 3.1 | all used functions exist (`FuncAnimation(cache_frame_data)`, `FancyArrowPatch.set_positions`, `RadioButtons(active)`, `solve_continuous_are`, SLSQP bounds, `splprep`) |

---

## 3. How it was tested (without a Foxy machine)

No Docker and no Foxy were available here, so I tested in a way that reproduces the Foxy environment:
* **Python 3.8.20** with Ubuntu 20.04's exact library versions: numpy 1.17.4, scipy 1.3.3, matplotlib 3.1.2, PyYAML 5.3.1.
* A **fake `rclpy` that exposes only Foxy's API**:
  * `init(*, args, context)` with no `signal_handler_options`;
  * a `rclpy.signals` without `SignalHandlerOptions`;
  * Foxy's QoS names;
  * an in-process topic bus.

| Test (real package code) | Result |
|---|---|
| Fake LIMO publishing PhaseSpace markers → `phasespace_bridge` → each controller's `main()` → `/cmd_vel` → fake LIMO, open S-curve | PID, LQR, LTV-MPC and NMPC each reach the path end (within 2 cm) and send a zero command last. RMS CTE 0.060 / 0.027 / 0.041 / 0.013 m. |
| `limo_test_suite` `main()` (circle, autostart) | 398 `/cmd_vel` messages, left 14° at 0.62 rad/s, final command zero |
| `interactive_builder`, `live_plotter` `main()` | open, draw (lab grid, robot glyph), close cleanly |
| `tools/check_deps.py` on Python 3.8 with `ROS_DISTRO=foxy` | detects Python 3.8 = Foxy, the Foxy Ctrl-C fallback, and the library versions; lists missing packages with apt commands |
| The same checks on this Humble machine | still all pass (no regression) |

Limits:
* The real Foxy `rclpy` C extension and DDS were not run; the APIs were taken from the Foxy source instead.
* The GUI test used matplotlib's Qt backend, because the test interpreter's Tk cannot be loaded by matplotlib 3.1. Ubuntu's `python3-tk` does not have this problem, and the drawing code is the same for every backend.

Run `tools/check_deps.py` on the real machine to confirm.

---

## 4. Should you install Python 3.10 on the Foxy machine? No.

**Compatibility check result: not compatible.**

* Foxy's `rclpy` is a C extension compiled for the Python 3.8 ABI. `/opt/ros/foxy/lib/python3.8/site-packages` cannot be imported by Python 3.10, so every node would fail with `ModuleNotFoundError: rclpy` or an ABI error.
* All the ROS message packages (`nav_msgs`, …) are likewise built for 3.8.
* Replacing `/usr/bin/python3` with 3.10 on Ubuntu 20.04 also breaks `apt`, `colcon` and other system tools, which are packaged for 3.8.
* The code itself no longer needs 3.10: it now runs on 3.8.

So no Python 3.10 install script is provided. Doing it would leave the system worse off without making the nodes run.

**If you need Python 3.10 / Humble anyway** (e.g. to match another machine):

| Option | Effort | Notes |
|---|---|---|
| **Stay on Foxy + Python 3.8** (recommended) | none | Works now; use `./tools/setup_ros2.sh` |
| Upgrade the machine to Ubuntu 22.04 + Humble | OS upgrade | Python 3.10 comes with it; the same `setup_ros2.sh` detects Humble |
| Run Humble in a Docker container on the Foxy machine | medium | Gives Python 3.10 + Humble without touching the host. Foxy ↔ Humble nodes on the same network usually interoperate for standard messages (same Fast DDS RMW), but it is not officially supported. Keep all nodes of this suite inside the container. |
