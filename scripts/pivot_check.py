#!/usr/bin/env python3
"""Which way does each wheel really roll for its direction line level?

Stack stopped (./start.sh stop). Starts the lidar only, then drives ONE
wheel at a time (bench mode, effort 15, 3 s) and measures the robot's pivot
with the lidar: left wheel forward = clockwise, right wheel forward =
anticlockwise. Prints which F/R level drives each wheel forward.

Usage: python3 scripts/pivot_check.py
"""
import glob
import math
import subprocess
import sys
import time
from pathlib import Path

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parent))
from direction_check import best_rotation, points  # noqa: E402

REST = 6.0     # s between wheels: the left driver refused a reversal 2 s after running


class Scan(Node):
    def __init__(self):
        super().__init__('pivot_check')
        self.msg = None
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'msg', m),
                                 qos_profile_sensor_data)

    def grab(self):
        self.msg = None
        end = time.time() + 10
        while self.msg is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return points(self.msg)


def main():
    lidar_port = (glob.glob('/dev/serial/by-id/usb-Silicon_Labs*') or ['/dev/ttyUSB0'])[0]
    # The robot's own bringup without the base controller: lidar + model, Mega free.
    lidar = subprocess.Popen(['ros2', 'launch', 'hospital_bringup', 'real_robot.launch.py',
                              'start_base:=false', f'lidar_port:={lidar_port}'],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = Scan()
    mega = serial.Serial(glob.glob('/dev/serial/by-id/usb-Arduino*')[0], 115200,
                         timeout=0.05, write_timeout=1)
    try:
        end = time.time() + 40
        while node.msg is None and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.msg is None:
            print('no lidar scans: is the lidar connected?')
            return
        print(f'lidar ready: {len(points(node.msg))} points per scan', flush=True)
        for side, name, fwd_sign in (('L', 'LEFT', -1), ('R', 'RIGHT', +1)):
            level = 0
            before = node.grab()
            end = time.time() + 3.0
            while time.time() < end:
                mega.write(f'PINS {side} B0 E1 F{level} A15\n'.encode())
                t = time.time()
                while time.time() - t < 0.4:
                    mega.read(8000)
                    rclpy.spin_once(node, timeout_sec=0.05)
            for _ in range(5):
                mega.write(b'PINS OFF\n')
                mega.read(8000)
                time.sleep(0.1)
            time.sleep(1.5)
            after = node.grab()
            if len(before) < 20 or len(after) < 20:
                print(f'{name}: too few lidar points ({len(before)}/{len(after)})')
                continue
            yaw, fit = best_rotation(before, after)
            rolled = 'FORWARD' if yaw * fwd_sign > 0 else 'BACKWARD'
            print(f'{name} wheel, F/R LOW: robot turned {yaw:+.0f} deg (fit {fit * 100:.1f} cm) '
                  f'-> wheel rolled {rolled}', flush=True)
            time.sleep(REST)
    finally:
        mega.write(b'PINS OFF\n')
        mega.close()
        lidar.terminate()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
