#!/usr/bin/env python3
"""Which way does the robot really turn when both wheels drive forward?

Records a lidar scan, drives straight forward slowly for SECS through the
obstacle stop, records another scan, and finds the rotation that matches the
two scans. Straight = wheel directions right; a turn of tens of degrees =
one wheel runs backwards (turning left -> LEFT wheel reversed, right -> RIGHT).

Usage: python3 scripts/direction_check.py   (stack running, ~1 m clear)
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import LaserScan

SPEED, SECS = 0.05, 3.0


def points(msg):
    r = np.asarray(msg.ranges, dtype=float)
    a = msg.angle_min + np.arange(r.size) * msg.angle_increment
    ok = np.isfinite(r) & (r > 0.3) & (r < 8.0)
    return np.column_stack((r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])))


def best_rotation(before, after):
    """Yaw (deg) and fit that best maps 'after' onto 'before' (small shift allowed)."""
    tree = cKDTree(before)
    best = (1e9, 0.0)
    for deg in np.arange(-180, 180, 1.0):
        t = math.radians(deg)
        rot = after @ np.array([[math.cos(t), math.sin(t)], [-math.sin(t), math.cos(t)]])
        for dx in (-0.2, -0.1, 0.0, 0.1, 0.2):
            d, _ = tree.query(rot + (dx, 0.0))
            score = float(np.median(d))
            if score < best[0]:
                best = (score, deg)
    return best[1], best[0]


class N(Node):
    def __init__(self):
        super().__init__('direction_check')
        self.scan = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m),
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel_smoothed', 10)

    def grab(self):
        self.scan = None
        while self.scan is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return points(self.scan)


def main():
    rclpy.init()
    n = N()
    time.sleep(1.0)
    before = n.grab()
    cmd = Twist()
    cmd.linear.x = SPEED
    end = time.monotonic() + SECS
    while time.monotonic() < end:
        n.pub.publish(cmd)
        rclpy.spin_once(n, timeout_sec=0.05)
    for _ in range(10):
        n.pub.publish(Twist())
        rclpy.spin_once(n, timeout_sec=0.05)
    time.sleep(1.5)
    after = n.grab()
    yaw, fit = best_rotation(before, after)
    print(f'points {len(before)}/{len(after)}; robot turned {yaw:+.0f} deg '
          f'(+ = left/anticlockwise), match error {fit * 100:.1f} cm')
    if abs(yaw) < 12:
        print('STRAIGHT: wheel directions are right')
    else:
        print(f'TURNED {"LEFT" if yaw > 0 else "RIGHT"}: the {"LEFT" if yaw > 0 else "RIGHT"} '
              'wheel runs backwards')


if __name__ == '__main__':
    main()
