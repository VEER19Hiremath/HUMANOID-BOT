#!/usr/bin/env python3
"""Check the Mega's wheel speed control through the base controller.

Wheels LIFTED. Sends /cmd_vel straight to base_controller (Nav2 idle) and
compares the commanded body speed / turn rate with what the wheel pulses
measured (/odom twist), then checks the stop time. Prints PASS/FAIL per case.

Usage: python3 scripts/wheel_speed_test.py
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

B = 0.4318
CASES = [  # name, v, w, seconds
    ('straight forward 0.06 m/s', 0.06, 0.0, 6.0),
    ('slow forward 0.03 m/s', 0.03, 0.0, 6.0),
    ('left arc', 0.05, 0.06, 6.0),
    ('right arc', 0.05, -0.06, 6.0),
    ('reverse 0.04 m/s', -0.04, 0.0, 6.0),
    ('forward after reverse', 0.06, 0.0, 7.0),
]


class Tester(Node):
    def __init__(self):
        super().__init__('wheel_speed_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.samples = []
        self.create_subscription(Odometry, '/odom', self._odom, 50)

    def _odom(self, msg):
        self.samples.append((time.monotonic(), msg.twist.twist.linear.x,
                             msg.twist.twist.angular.z))

    def drive(self, v, w, secs):
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        end = time.monotonic() + secs
        while time.monotonic() < end:
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)

    def idle(self, secs):
        end = time.monotonic() + secs
        while time.monotonic() < end:
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)


def expected(v, w):
    """Body speed/turn the base controller should produce (its arc limits)."""
    sys.path.insert(0, '')
    from wheel_odometry.base_controller import wheel_speeds
    left, right = wheel_speeds(v, w, B)
    return 0.5 * (left + right), (right - left) / B


def main():
    rclpy.init()
    node = Tester()
    node.idle(1.0)
    bad = 0
    for name, v, w, secs in CASES:
        ev, ew = expected(v, w)
        node.samples.clear()
        node.drive(v, w, secs)
        t0 = node.samples[0][0] if node.samples else time.monotonic()
        steady = [s for s in node.samples if s[0] > t0 + secs - 3.0]
        mv = sum(s[1] for s in steady) / max(len(steady), 1)
        mw = sum(s[2] for s in steady) / max(len(steady), 1)
        ok_v = abs(mv - ev) <= 0.10 * abs(ev) + 0.003
        ok_w = abs(mw - ew) <= 0.15 * abs(ew) + 0.01
        print(f'{"PASS" if ok_v and ok_w else "FAIL"} {name:<26} speed {mv:+.3f} '
              f'(want {ev:+.3f})  turn {mw:+.3f} (want {ew:+.3f}) rad/s', flush=True)
        bad += not (ok_v and ok_w)
    # Stop time: from the zero command to the wheels standing still.
    node.drive(0.06, 0.0, 4.0)
    node.samples.clear()
    t_stop = time.monotonic()
    node.idle(3.0)
    moving = [s for s in node.samples if abs(s[1]) > 0.003]
    stop_s = (moving[-1][0] - t_stop) if moving else 0.0
    ok = stop_s < 1.0
    print(f'{"PASS" if ok else "FAIL"} stop from 0.06 m/s: wheels still after {stop_s:.2f} s',
          flush=True)
    bad += not ok
    node.destroy_node()
    rclpy.shutdown()
    print(f'\n{bad} failure(s)')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
