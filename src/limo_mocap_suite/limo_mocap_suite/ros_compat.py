# #############################################################################
# [FILE]   ros_compat.py
# -----------------------------------------------------------------------------
# FILE  : src/limo_mocap_suite/limo_mocap_suite/ros_compat.py
# ROLE  : Small shims so the package runs on ROS 2 Foxy (Python 3.8) AND
#         Humble (Python 3.10) without changes.
#
# Differences handled:
#   rclpy.init(signal_handler_options=...) and rclpy.signals.SignalHandlerOptions
#   exist only from Humble on. On Humble the default SIGINT handler shuts the
#   ROS context down before our cleanup runs (the final zero Twist would fail),
#   so we disable it there. Foxy's handler only wakes the executor and raises
#   KeyboardInterrupt (context stays alive), so plain rclpy.init is correct.
#
# TAG LEGEND (grep for these to jump / skip):
#   [CORE]   control / estimation math that decides robot motion
#   [ROS-IO] ROS 2 publishers, subscribers, callbacks, timers
#   [UI]     Matplotlib GUI widgets and mouse handlers (safe to skip)
#   [LOG]    console prints and CSV telemetry (safe to skip)
#   [LIFE]   node construction, startup and shutdown plumbing
#   [UTIL]   small stateless helpers
# #############################################################################

import rclpy

try:                                    # Humble and newer
    from rclpy.signals import SignalHandlerOptions
except ImportError:                     # Foxy / Galactic
    SignalHandlerOptions = None


# =============================================================================
# [LIFE]   init_ros(args=None)
# -----------------------------------------------------------------------------
# rclpy.init that keeps the ROS context alive on Ctrl-C on every distro:
#   Humble+: rclpy.init(args, signal_handler_options=SignalHandlerOptions.NO)
#   Foxy   : rclpy.init(args)  (its SIGINT handling already behaves this way)
# Ctrl-C then raises KeyboardInterrupt in spin() and the node's finally block
# can still publish (zero Twist) before shutting down.
# =============================================================================
def init_ros(args=None):
    if SignalHandlerOptions is not None:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    else:
        rclpy.init(args=args)
