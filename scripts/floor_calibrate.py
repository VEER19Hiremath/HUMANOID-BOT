#!/usr/bin/env python3
"""Calibrate wheel odometry on the floor against the lidar. No tape needed.

Drives straight (equal wheel commands) for DIST metres through the obstacle
stop (collision_monitor), then compares what odometry thinks happened with
what really happened, measured by matching the lidar scan before and after
(ICP). From that it solves the two pulse scales:

  distance_per_pulse          metres per LEFT wheel pulse
  right_encoder_multiplier    right pulse length / left pulse length

A wrong ratio makes odometry believe the robot turns when it doesn't, so
Nav2 "corrects" and the robot drives in circles.

It also checks which way the lidar faces: if the scans only match with the
motion reversed, the lidar is mounted backwards (yaw 180 deg) and the obstacle
stop is watching the wrong side.

--tape: the lidar can't see enough of the room (it sits inside the body),
so measure by hand instead: put a tape mark on the floor at each wheel's hub
(its contact point), run, then mark the hubs again and measure how far the
MIDPOINT between them moved forward and sideways (a front-edge mark swings
sideways in any turn and spoils the result). The script asks for both and writes the result to
config/odometry.yaml (src and install), used from the next ./start.sh.

Needs: ./start.sh running, no active goal, ~2 m clear ahead (or behind for a
negative distance), someone at the battery switch.
Usage: python3 scripts/floor_calibrate.py [--tape] [metres, negative = reverse]
"""

import math
import re
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan

SPEED = 0.06                 # m/s, the cruise speed
WHEEL_BASE = 0.4318          # base_controller wheel_base


def scan_points(msg):
    r = np.asarray(msg.ranges, dtype=np.float64)
    a = msg.angle_min + np.arange(r.size) * msg.angle_increment
    ok = np.isfinite(r) & (r > 0.25) & (r < 8.0)
    return np.column_stack((r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])))


def icp(src, dst, guess, iters=60):
    """Rigid 2-D transform (x, y, yaw) mapping src points onto dst."""
    x, y, t = guess
    tree = cKDTree(dst)
    gate = 0.5
    for _ in range(iters):
        c, s = math.cos(t), math.sin(t)
        moved = src @ np.array([[c, s], [-s, c]]) + (x, y)
        d, idx = tree.query(moved)
        keep = d < gate
        if keep.sum() < 30:
            break
        p, q = moved[keep], dst[idx[keep]]
        pm, qm = p.mean(0), q.mean(0)
        h = (p - pm).T @ (q - qm)
        dt = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
        cr, sr = math.cos(dt), math.sin(dt)
        rot = np.array([[cr, -sr], [sr, cr]])
        shift = qm - rot @ pm
        # compose the increment with the current estimate
        nx, ny = rot @ (x, y) + shift
        x, y, t = nx, ny, t + dt
        gate = max(0.08, gate * 0.85)
    c, s = math.cos(t), math.sin(t)
    moved = src @ np.array([[c, s], [-s, c]]) + (x, y)
    d, _ = tree.query(moved)
    return (x, y, t), float(np.median(d)), float((d < 0.1).mean())


class Calib(Node):
    def __init__(self):
        super().__init__('floor_calibrate')
        self.scan = None
        self.odom = []
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, 50)
        # Into the collision monitor, so the obstacle stop still applies.
        self.pub = self.create_publisher(Twist, '/cmd_vel_smoothed', 10)

    def _on_scan(self, msg):
        self.scan = msg

    def _on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.odom.append((p.x, p.y, 2 * math.atan2(q.z, q.w)))

    def wait(self, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def still_scan(self, n=5):
        """Average of n scans' points is noisy; take the median-range scan."""
        scans = []
        while len(scans) < n:
            self.scan = None
            while self.scan is None:
                rclpy.spin_once(self, timeout_sec=0.1)
            scans.append(np.asarray(self.scan.ranges, dtype=np.float64))
        msg = self.scan
        msg.ranges = list(np.nanmedian(np.vstack(scans), axis=0))
        return scan_points(msg)

    def params(self):
        cli = self.create_client(GetParameters, '/base_controller/get_parameters')
        cli.wait_for_service(timeout_sec=10)
        names = ['distance_per_pulse', 'right_encoder_multiplier', 'wheel_base']
        fut = cli.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10)
        return [v.double_value for v in fut.result().values]


def odom_motion(samples):
    """Path length and heading change from the odom stream."""
    length = 0.0
    for (x0, y0, _), (x1, y1, _) in zip(samples, samples[1:]):
        length += math.hypot(x1 - x0, y1 - y0)
    turn = 0.0
    for (_, _, a), (_, _, b) in zip(samples, samples[1:]):
        turn += math.atan2(math.sin(b - a), math.cos(b - a))
    return length, turn


def arc_length(chord, turn):
    return chord if abs(turn) < 1e-3 else chord * (turn / 2) / math.sin(turn / 2)


def solve(so, to, s_true, t_true, k0, m0, b):
    """Scales that make odometry match the true motion.

    Pulses with the current scales: k0*PL = so - b*to/2, k0*m0*PR = so + b*to/2
    New scales: k*PL = s - b*t/2, k*m*PR = s + b*t/2.
    """
    pl = (so - b * to / 2) / k0
    pr = (so + b * to / 2) / (k0 * m0)
    k = (s_true - b * t_true / 2) / pl
    m = (s_true + b * t_true / 2) / (k * pr)
    return k, m, pl, pr


def write_calibration(k, m):
    root = Path(__file__).resolve().parent.parent
    for f in (root / 'src/hospital_bringup/config/odometry.yaml',
              root / 'install/hospital_bringup/share/hospital_bringup/config/odometry.yaml'):
        if not f.exists():
            continue
        text = f.read_text()
        text = re.sub(r'distance_per_pulse: .*', f'distance_per_pulse: {k:.6f}', text)
        text = re.sub(r'right_encoder_multiplier: .*',
                      f'right_encoder_multiplier: {m:.4f}', text)
        f.write_text(text)
        print(f'wrote {f.relative_to(root)}')


def tape(node, dist, k0, m0, b):
    samples = list(node.odom)
    so, to = odom_motion(samples)
    if so < 0.2:
        print(f'odometry moved only {so:.2f} m: wheels did not turn. Check base.log.')
        return 1
    so = math.copysign(so, dist)
    print(f'odometry: {so:+.3f} m, turned {math.degrees(to):+.1f} deg')
    fwd = float(input('Measured at the midpoint between the hubs: FORWARD (cm)? ')) / 100.0
    side = float(input('                  and SIDEWAYS (cm, + = to the robot\'s left)? ')) / 100.0
    if dist < 0:
        fwd = -abs(fwd)
    # Constant-curvature arc from the start pose to (fwd, side).
    t_true = 2 * math.atan2(side, abs(fwd)) * (1 if dist > 0 else -1)
    s_true = math.copysign(arc_length(math.hypot(fwd, side), t_true), dist)
    print(f'measured: {s_true:+.3f} m, turned {math.degrees(t_true):+.1f} deg')
    k, m, _, _ = solve(so, to, s_true, t_true, k0, m0, b)
    print(f'\nNEW distance_per_pulse       = {k:.6f}   (was {k0:.6f})')
    print(f'NEW right_encoder_multiplier = {m:.4f}   (was {m0:.4f})')
    if not (0.5 * k0 < k < 2 * k0 and 0.7 < m < 1.4):
        print('These are far from the old values: check the measurement, nothing written.')
        return 1
    write_calibration(k, m)
    print('Restart (./start.sh stop; ./start.sh) to use them.')
    return 0


def main():
    args = sys.argv[1:]
    use_tape = '--tape' in args
    args = [a for a in args if a != '--tape']
    dist = float(args[0]) if args else (1.5 if use_tape else 1.0)
    rclpy.init()
    node = Calib()
    k0, m0, b = node.params()
    b = b or WHEEL_BASE
    print(f'current: distance_per_pulse={k0:.5f} right_encoder_multiplier={m0:.4f} '
          f'wheel_base={b:.4f}')

    if use_tape:
        input(f'Tape-mark the floor at both wheel hubs, clear {abs(dist) + 0.5:.1f} m, '
              'stand at the battery switch, then press Enter to drive ...')
    else:
        start = node.still_scan()
    node.odom.clear()
    print(f'driving straight {dist:+.2f} m ...', flush=True)
    speed = math.copysign(SPEED if dist > 0 else 0.04, dist)   # reverse slower
    t_end = time.monotonic() + abs(dist) / abs(speed)
    cmd = Twist()
    cmd.linear.x = speed
    try:
        while time.monotonic() < t_end:
            node.pub.publish(cmd)
            node.wait(0.05)
    finally:
        for _ in range(10):
            node.pub.publish(Twist())
            node.wait(0.05)
    node.wait(2.0)
    if use_tape:
        return tape(node, dist, k0, m0, b)
    samples = list(node.odom)
    end = node.still_scan()

    so, to = odom_motion(samples)
    np.savez('/tmp/floor_calibrate_last.npz', start=start, end=end,
             odom=np.array(samples))
    if so < 0.2:
        print(f'odometry moved only {so:.2f} m: wheels did not turn (stall cut, '
              'obstacle stop, or battery off). Check base.log.')
        return 1
    so = math.copysign(so, dist)
    # Odom's guess of the motion, as the start for scan matching. The end
    # scan, moved by the robot's motion, lands on the start scan.
    ox = samples[-1][0] - samples[0][0]
    oy = samples[-1][1] - samples[0][1]
    h0 = samples[0][2]
    gx = math.cos(-h0) * ox - math.sin(-h0) * oy
    gy = math.sin(-h0) * ox + math.cos(-h0) * oy
    # Try the lidar facing forward (as in the URDF) and facing backwards, and
    # seed with odometry scaled 0.5-4x: uncalibrated odometry was off by ~2.5x
    # on the floor, too far for a single seed to converge.
    def best(sign):
        tries = [icp(end, start, (sign * gx * sc, sign * gy * sc, to + dth))
                 for sc in np.arange(0.5, 4.01, 0.25)
                 for dth in np.radians([-15.0, 0.0, 15.0])]
        return min(tries, key=lambda r: r[1] - 0.05 * r[2])
    fwd = best(1)
    back = best(-1)
    print(f'scan match, lidar facing forward : median error {fwd[1] * 100:.1f} cm, '
          f'{fwd[2] * 100:.0f}% matched')
    print(f'scan match, lidar facing backward: median error {back[1] * 100:.1f} cm, '
          f'{back[2] * 100:.0f}% matched')
    if back[1] < fwd[1] and back[2] > fwd[2]:
        print('\n*** The LIDAR IS MOUNTED BACKWARDS: set yaw 3.14159 on the laser '
              'joint. Until then the obstacle stop watches behind the robot. ***\n')
        (tx, ty, tt), med, frac = back
        tx, ty = -tx, -ty   # robot motion = reverse of the backward-lidar motion
    else:
        (tx, ty, tt), med, frac = fwd
    if med > 0.06 or frac < 0.5:
        print('scan match is poor (too few walls in view, or people moving). '
              'Try again facing a wall or furniture 1-4 m away.')
        return 1

    chord = math.hypot(tx, ty)
    s_true = math.copysign(arc_length(chord, tt), dist)
    print(f'odometry: {so:.3f} m, turned {math.degrees(to):+.1f} deg')
    print(f'lidar   : {s_true:.3f} m, turned {math.degrees(tt):+.1f} deg '
          f'(dx {tx:+.3f}, dy {ty:+.3f})')

    # Pulses from odom with the current scales, then the scales that make
    # odom match the lidar:  k*PL = s - b*t/2,  k*m*PR = s + b*t/2
    k, m, pl, pr = solve(so, to, s_true, tt, k0, m0, b)
    radius = s_true / tt if abs(tt) > 1e-3 else float('inf')
    print(f'pulses  : left {pl:.0f}, right {pr:.0f}')
    print(f'with equal commands the robot really arcs with radius {radius:.1f} m')
    print(f'\nNEW distance_per_pulse       = {k:.5f}   (was {k0:.5f})')
    print(f'NEW right_encoder_multiplier = {m:.4f}   (was {m0:.4f})')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
