# #############################################################################
# [FILE]   plot_helpers.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/plot_helpers.py
# ROLE  : Shared Matplotlib drawing helpers for the GUIs (interactive_builder,
#         live_plotter, limo_test_suite): lab canvas + projector styling
#         (thick red axes, 1 m grid, large labels), start-frame transform,
#         robot glyph with an unambiguous heading, obstacle circles.
#
# Not a node. Mostly [UI]; StartFrame is the only math (same transform as the
# controllers' odom_cb, so the GUIs show what the controllers see).
# All three GUIs show the same view (parameter 'view'), so a path looks the same
# in each. Only the display is rotated: ROS messages always carry real X, Y.
# Heading convention: robot body +x = vector from the REAR marker (id 1) to
# the FRONT marker (id 0), exactly as computed in phasespace_bridge.py.
# Frame: mocap frame is ROS right-handed (+x forward, +y left, yaw counter-clockwise).
#
# TAG LEGEND (grep for these to jump / skip):
#   [CORE]   control / estimation math that decides robot motion
#   [ROS-IO] ROS 2 publishers, subscribers, callbacks, timers
#   [UI]     Matplotlib GUI widgets and mouse handlers (safe to skip)
#   [LOG]    console prints and CSV telemetry (safe to skip)
#   [LIFE]   node construction, startup and shutdown plumbing
#   [UTIL]   small stateless helpers
# #############################################################################

import math
import signal
import threading

import rclpy
from limo_mocap_suite.ros_compat import init_ros
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.ticker import FuncFormatter, MultipleLocator

# Lab floor covered by the PhaseSpace system (metres). Overridable per node with
# the ROS parameters lab_x / lab_y (see declare_lab_params).
from limo_mocap_suite.track_tools import LAB_X, LAB_Y  # noqa: E402  mocap area, one place
AXIS_COLOR = 'red'    # colour of the thick X / Y axes through (0, 0)

# LIMO footprint used for the robot glyph (metres)
BODY_LENGTH = 0.32
BODY_WIDTH = 0.22
MARKER_OFFSET = 0.12   # drawn distance of the front / rear marker from the centre

WALL_RADIUS = 100.0    # obstacles at least this big are drawn as walls


# =============================================================================
# [UTIL]   quat_to_yaw(q) -> float
# -----------------------------------------------------------------------------
# Planar yaw (rad, counter-clockwise from +x) from a geometry_msgs Quaternion
# (same formula as the controllers). Only valid for a robot that stays flat.
# =============================================================================
def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# =============================================================================
# [LIFE]   init_gui_ros(args)
# -----------------------------------------------------------------------------
# args: command-line args for rclpy. init_ros (plain Python SIGINT handling on
# Foxy and Humble), and Ctrl-C
# mapped to "close all windows". plt.show() then returns normally and the
# node's cleanup (zero Twist, destroy, shutdown) runs, also under ros2 launch.
# The GUIs' timers (5-10 Hz) let Python see the signal while Tk is idle.
# =============================================================================
def init_gui_ros(args=None):
    init_ros(args)
    signal.signal(signal.SIGINT, lambda *_: plt.close('all'))


# =============================================================================
# [LIFE]   start_spin_thread(node) -> Thread
# -----------------------------------------------------------------------------
# node: the rclpy Node to spin. Spins it in a daemon thread (Matplotlib owns the main thread). The
# exception raised when ROS shuts down underneath the spin is swallowed, so
# closing a window prints no traceback.
# =============================================================================
def start_spin_thread(node):
    # (nested) thread body: spin until ROS shuts down
    def run():
        try:
            rclpy.spin(node)
        except Exception:  # noqa: BLE001  (shutdown while spinning)
            pass
    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread


# =============================================================================
# [UTIL]   declare_lab_params(node) -> (lab_x, lab_y)
# -----------------------------------------------------------------------------
# Declares and reads the lab_x / lab_y parameters ([min, max] in metres,
# defaults LAB_X / LAB_Y) on a GUI node. Returns ((x_min, x_max), (y_min, y_max)).
# Raises on a malformed range.
# =============================================================================
def declare_lab_params(node):
    node.declare_parameter('lab_x', list(LAB_X))
    node.declare_parameter('lab_y', list(LAB_Y))
    out = []
    for name in ('lab_x', 'lab_y'):
        v = [float(a) for a in node.get_parameter(name).value]
        if len(v) != 2 or v[0] >= v[1]:
            raise ValueError(f"'{name}' must be [min, max] with min < max, got {v}")
        out.append((v[0], v[1]))
    return out[0], out[1]


# =============================================================================
# [UTIL]   declare_view_param(node) -> behind_view (bool)
# -----------------------------------------------------------------------------
# Declares and reads the 'view' parameter of a GUI node. True = behind view:
#   'behind'  (default): +X (forward) up, +Y (left) left
#   'topdown'          : +X right, +Y up
# Raises on any other value.
# =============================================================================
def declare_view_param(node):
    node.declare_parameter('view', 'behind')
    view = str(node.get_parameter('view').value)
    if view not in ('behind', 'topdown'):
        raise ValueError(f"'view' must be 'behind' or 'topdown', got '{view}'")
    return view == 'behind'


# =============================================================================
# [UI]     style_lab_axes(ax, lab_x, lab_y, title, behind_view=False)
# -----------------------------------------------------------------------------
# Projector-friendly lab grid (ax: Matplotlib axes, lab_x / lab_y: (min, max) in m):
#   - limits = lab extents, equal aspect
#   - 1 m major grid (dark) + 0.5 m minor grid (light)
#   - thick RED lines on the X and Y axes (through 0) with '+X' / '+Y' labels
#   - large tick and axis labels
# behind_view=False: +X right, +Y up (top-down lab map).
# behind_view=True : view from BEHIND the robot: +X (forward) points UP and
#                    +Y (left) points LEFT. Plot data with to_view(); the
#                    horizontal tick labels show real Y values.
# =============================================================================
def style_lab_axes(ax, lab_x, lab_y, title, behind_view=False):
    if behind_view:
        ax.set_xlim(-lab_y[1], -lab_y[0])
        ax.set_ylim(*lab_x)
        ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{-v + 0.0:g}'))
        ax.set_xlabel('Y (m)   <- left   |   right ->', fontsize=13, fontweight='bold')
        ax.set_ylabel('X (m)   forward ->', fontsize=13, fontweight='bold')
        ax.format_coord = lambda u, v: f'X={v:.2f} m  Y={-u:.2f} m'   # toolbar read-out: real X, Y
    else:
        ax.set_xlim(*lab_x)
        ax.set_ylim(*lab_y)
        ax.set_xlabel('X (m)', fontsize=13, fontweight='bold')
        ax.set_ylabel('Y (m)', fontsize=13, fontweight='bold')
    ax.set_aspect('equal')
    ax.set_title(title, fontsize=15, fontweight='bold')
    for axis in (ax.xaxis, ax.yaxis):
        axis.set_major_locator(MultipleLocator(1.0))
        axis.set_minor_locator(MultipleLocator(0.5))
    ax.tick_params(labelsize=13)
    ax.grid(True, which='major', color='0.45', lw=1.0)
    ax.grid(True, which='minor', color='0.8', lw=0.6, ls=':')
    ax.axhline(0.0, color=AXIS_COLOR, lw=3.5, zorder=2)
    ax.axvline(0.0, color=AXIS_COLOR, lw=3.5, zorder=2)
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    style = dict(color=AXIS_COLOR, fontsize=16, fontweight='bold', zorder=8,
                 bbox=dict(facecolor='white', edgecolor=AXIS_COLOR, alpha=0.85))
    if behind_view:
        ax.text(0.05, y1 - 0.1, '+X (forward)', va='top', ha='left', **style)
        ax.text(x0 + 0.1, 0.1, '+Y (left)', va='bottom', ha='left', **style)
    else:
        ax.text(x1 - 0.1, 0.1, '+X', va='bottom', ha='right', **style)
        ax.text(0.1, y1 - 0.6, '+Y', va='top', ha='left', **style)   # below the status text
    ax.text(0.0, 0.0, ' (0,0)', color=AXIS_COLOR, fontsize=12, fontweight='bold',
            va='top', ha='left', zorder=8)


# =============================================================================
# [UI]     lab_legend(ax, behind_view=False)
# -----------------------------------------------------------------------------
# Draws the legend clear of the '+X' label and the status text, for either view.
# Call it again after adding new labelled artists (e.g. obstacles).
# =============================================================================
def lab_legend(ax, behind_view=False):
    if behind_view:
        ax.legend(loc='upper left', bbox_to_anchor=(0.0, 0.95))
    else:
        ax.legend(loc='upper right')


# =============================================================================
# [UTIL]   to_view(x, y, yaw=0.0, behind_view=False) -> (u, v, yaw_view)
# -----------------------------------------------------------------------------
# Maps lab / robot-frame coordinates (m, rad) to plot coordinates. Identity for the
# top-down view; for the behind view (+X up, +Y left): u = -y, v = x,
# yaw_view = yaw + pi/2.
# =============================================================================
def to_view(x, y, yaw=0.0, behind_view=False):
    if behind_view:
        return -y, x, yaw + math.pi / 2.0
    return x, y, yaw


# =============================================================================
# [UTIL]   from_view(u, v, behind_view=False) -> (x, y)
# -----------------------------------------------------------------------------
# Inverse of to_view: canvas coordinates back to real X, Y. The builder must
# use this before publishing what was drawn. For the behind view: x = v, y = -u.
# =============================================================================
def from_view(u, v, behind_view=False):
    if behind_view:
        return v, -u
    return u, v


# =============================================================================
# [CORE]   class StartFrame
# -----------------------------------------------------------------------------
# Mirrors the controllers' re-zeroing (rezero_odom) so the GUIs draw in the
# controller frame.
# rezero=True : latch() stores the FIRST pose given; apply() then expresses
#               poses / points relative to it (translate + rotate).
# rezero=False: identity (absolute PhaseSpace frame).
# =============================================================================
class StartFrame:
    # =========================================================================
    # [LIFE]   __init__(rezero=True)
    # -------------------------------------------------------------------------
    # rezero: same value as the controllers' rezero_odom parameter. With False
    # the origin is fixed at (0, 0, 0) and the frame is ready at once.
    # =========================================================================
    def __init__(self, rezero=True):
        self.rezero = rezero
        self.origin = None if rezero else (0.0, 0.0, 0.0)    # (x, y, yaw) of the frame in mocap coordinates

    # =========================================================================
    # [CORE]   latch(x, y, yaw)
    # -------------------------------------------------------------------------
    # Stores the origin once (first call wins), like the controllers' odom_cb.
    # =========================================================================
    def latch(self, x, y, yaw):
        if self.origin is None:
            self.origin = (x, y, yaw)

    # =========================================================================
    # [UTIL]   ready -> bool  (property)
    # -------------------------------------------------------------------------
    # True once the origin is known; apply() must not be called before.
    # =========================================================================
    @property
    def ready(self):
        return self.origin is not None

    # =========================================================================
    # [CORE]   apply(x, y, yaw=0.0) -> (x', y', yaw')
    # -------------------------------------------------------------------------
    # Mocap pose (m, m, rad) -> controller-frame pose.
    # p' = R(-yaw0) (p - p0),  yaw' = wrap(yaw - yaw0).
    # =========================================================================
    def apply(self, x, y, yaw=0.0):
        x0, y0, yaw0 = self.origin
        dx, dy = x - x0, y - y0
        c, s = math.cos(yaw0), math.sin(yaw0)
        return (c * dx + s * dy, -s * dx + c * dy,
                math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0)))


# =============================================================================
# [UI]     class RobotGlyph
# -----------------------------------------------------------------------------
# Robot drawn so the heading can never be misread:
#   body    : grey rectangle with the LIMO footprint
#   heading : thick arrow line from REAR marker (M1) to FRONT marker (M0)
#   M0 dot  : red  = FRONT marker (id 0)
#   M1 dot  : blue = REAR marker  (id 1)
# update() moves all parts; artists() returns them for the animation.
# =============================================================================
class RobotGlyph:
    # =========================================================================
    # [UI]     __init__(ax, label='LIMO')
    # -------------------------------------------------------------------------
    # Adds the (hidden) body, arrow and two marker dots to ax; they show up at
    # the first update(). label is the legend text of the body.
    # =========================================================================
    def __init__(self, ax, label='LIMO'):
        self.body = patches.Polygon([[0, 0]] * 4, closed=True, facecolor='0.85',
                                    edgecolor='0.3', lw=1.2, zorder=5, label=label)
        ax.add_patch(self.body)
        self.heading = patches.FancyArrowPatch((0, 0), (0, 0), arrowstyle='-|>',
                                               mutation_scale=18, lw=2.5,
                                               color='black', zorder=6)
        ax.add_patch(self.heading)
        self.front, = ax.plot([], [], 'o', color='red', ms=7, zorder=7,
                              label='M0 front marker')
        self.rear, = ax.plot([], [], 'o', color='royalblue', ms=7, zorder=7,
                             label='M1 rear marker')
        self.set_visible(False)

    # =========================================================================
    # [UI]     update(x, y, yaw)
    # -------------------------------------------------------------------------
    # x, y (m) and yaw (rad) in plot coordinates (use to_view first). Places the
    # body rectangle, the M1 -> M0 arrow and both marker dots.
    # =========================================================================
    def update(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        hl, hw = BODY_LENGTH / 2.0, BODY_WIDTH / 2.0
        corners = [(hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw)]    # body corners, robot frame
        self.body.set_xy([(x + c * px - s * py, y + s * px + c * py) for px, py in corners])
        fx, fy = x + c * MARKER_OFFSET, y + s * MARKER_OFFSET
        rx, ry = x - c * MARKER_OFFSET, y - s * MARKER_OFFSET
        self.heading.set_positions((rx, ry), (x + c * (hl + 0.08), y + s * (hl + 0.08)))
        self.front.set_data([fx], [fy])
        self.rear.set_data([rx], [ry])
        self.set_visible(True)

    # =========================================================================
    # [UI]     set_visible(visible)
    # -------------------------------------------------------------------------
    # Shows or hides all four parts together (hidden until the first pose).
    # =========================================================================
    def set_visible(self, visible):
        for a in self.artists():
            a.set_visible(visible)

    # =========================================================================
    # [UI]     artists() -> list of Matplotlib artists
    # -------------------------------------------------------------------------
    # The four parts, for the animation / blitting code.
    # =========================================================================
    def artists(self):
        return [self.body, self.heading, self.front, self.rear]


# =============================================================================
# [UI]     draw_obstacles(ax, flat, frame, old=(), behind_view=False) -> list[Patch]
# -----------------------------------------------------------------------------
# Removes the previously drawn obstacle patches (old) and draws the flat
# [x, y, r, ...] map (m, mocap frame) transformed by `frame` (a ready
# StartFrame) and to_view:
#   r <  WALL_RADIUS : filled red circle ('Obstacle')
#   r >= WALL_RADIUS : dashed black circle edge (a wall; looks straight)
# Returns the new patches so the caller can remove them next time.
# =============================================================================
def draw_obstacles(ax, flat, frame, old=(), behind_view=False):
    for p in old:
        p.remove()
    new = []
    first_obs, first_wall = True, True
    for i in range(0, len(flat) - 2, 3):
        ox, oy, _ = frame.apply(flat[i], flat[i + 1])
        ox, oy, _ = to_view(ox, oy, behind_view=behind_view)
        r = flat[i + 2]
        if r < WALL_RADIUS:    # first of each kind gets the legend label
            p = patches.Circle((ox, oy), r, facecolor='tomato', edgecolor='darkred',
                               alpha=0.6, zorder=3, label='Obstacle' if first_obs else None)
            first_obs = False
        else:
            p = patches.Circle((ox, oy), r, fill=False, edgecolor='black', ls='--', lw=1.5,
                               zorder=3, label='Wall' if first_wall else None)
            first_wall = False
        ax.add_patch(p)
        new.append(p)
    return new
