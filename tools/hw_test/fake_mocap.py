"""Test only: turns the stand-in robot's truth into marker frames, with chosen faults."""
import math, random, sys
import rclpy
from nav_msgs.msg import Odometry
from phasespace_msgs.msg import Marker, Markers
mode = sys.argv[1] if len(sys.argv) > 1 else 'normal'
rclpy.init(); n = rclpy.create_node('fake_mocap'); pub = n.create_publisher(Markers, '/phasespace/markers', 1)
S = {'p': None, 'k': 0}
def cb(m):
    S['p'] = m
def mk(i, x, y):
    if mode == 'mirror':
        y = -y
    return Marker(id=i, x=float(x + random.gauss(0, .0005)), y=float(y + random.gauss(0, .0005)), z=0.1, cond=1.0)
def tick():
    m = S['p']
    if m is None: return
    S['k'] += 1
    p = m.pose.pose; q = p.orientation
    yaw = 2 * math.atan2(q.z, q.w)
    fx, fy = p.position.x + .1 * math.cos(yaw), p.position.y + .1 * math.sin(yaw)
    rx, ry = p.position.x - .1 * math.cos(yaw), p.position.y - .1 * math.sin(yaw)
    f, r = (1, 0) if mode == 'swap' else (0, 1)
    if mode == 'glitch' and S['k'] % 200 == 0:  fx += .6                # ghost: front marker jumps
    if mode == 'glitch' and S['k'] % 330 == 0:  fx, fy, rx, ry = rx, ry, fx, fy   # swap for a frame
    if mode == 'glitch' and 600 < S['k'] % 1500 < 640: return            # occlusion 0.4 s
    pub.publish(Markers(markers=[mk(f, fx, fy), mk(r, rx, ry)]))
n.create_subscription(Odometry, '/sim_truth', cb, 10); n.create_timer(0.01, tick); rclpy.spin(n)
