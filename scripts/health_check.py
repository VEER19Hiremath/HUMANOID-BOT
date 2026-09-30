#!/usr/bin/env python3
"""Stack health in one shot: lidar, odometry, localization, map, obstacle stop.

Talks to the nodes directly (rclpy). The `ros2 lifecycle get` / `ros2 topic`
CLI goes through the ros2 daemon and reported false "not active" warnings on
the loaded Pi. Exit 0 when everything needed to drive is up.

Usage: scripts/health_check.py [--mapping]   (--mapping: SLAM mode checks)
"""

import math
import sys
import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

LISTEN_S = 6.0


def lifecycle_state(node, name):
    client = node.create_client(GetState, f'/{name}/get_state')
    if not client.wait_for_service(timeout_sec=8.0):
        return 'missing'
    future = client.call_async(GetState.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
    result = future.result()
    return result.current_state.label if result else 'no reply'


def lidar_view(msg):
    """How much of the room the lidar sees, and whether it sees ahead.

    Floor test 2026-09-30: 99 of 720 beams returned and none ahead, yet
    'Lidar: OK'. The obstacle stop then can't see what's in front.
    """
    n = len(msg.ranges)
    valid = ahead = ahead_valid = 0
    for i, r in enumerate(msg.ranges):
        a = math.degrees(msg.angle_min + i * msg.angle_increment)
        ok = math.isfinite(r) and msg.range_min < r < msg.range_max
        valid += ok
        if -45.0 <= a <= 45.0:
            ahead += 1
            ahead_valid += ok
    share = valid / n if n else 0.0
    front = ahead_valid / ahead if ahead else 0.0
    detail = f'{share * 100:.0f}% of beams return, {front * 100:.0f}% ahead'
    if share < 0.25 or front < 0.10:
        return (f'{"Lidar view:":<15} NOTE  ({detail}): nothing in range ahead. Normal '
                'on an open floor (walls > 12 m away); if there ARE things around, '
                'check the lidar is uncovered and level. The obstacle stop still '
                'sees anything that comes near.')
    return f'{"Lidar view:":<15} OK  ({detail})'


def main():
    mapping = '--mapping' in sys.argv
    rclpy.init()
    node = Node('health_check')
    counts = {'scan': 0, 'odom': 0, 'map': 0}
    frames = set()

    def tf(msg):
        for t in msg.transforms:
            frames.add((t.header.frame_id, t.child_frame_id))

    last_scan = []

    def on_scan(msg):
        counts['scan'] += 1
        last_scan[:] = [msg]

    node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(
        Odometry, '/odom', lambda m: counts.__setitem__('odom', counts['odom'] + 1), 10)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(
        OccupancyGrid, '/map', lambda m: counts.__setitem__('map', counts['map'] + 1),
        latched)
    node.create_subscription(TFMessage, '/tf', tf, 50)
    # Drawn maps: map->odom is a static transform (start pose), not AMCL.
    static_tf = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(TFMessage, '/tf_static', tf, static_tf)
    end = time.monotonic() + LISTEN_S
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    checks = [
        ('Lidar', counts['scan'] > 0, f'{counts["scan"] / LISTEN_S:.0f} scans/s'),
        ('Odometry', counts['odom'] > 0 and ('odom', 'base_footprint') in frames,
         f'{counts["odom"] / LISTEN_S:.0f} msgs/s'),
        ('Localization', ('map', 'odom') in frames,
         'map frame published' if ('map', 'odom') in frames else 'no map frame'),
        ('Map', counts['map'] > 0, 'received' if counts['map'] else 'not received'),
    ]
    if not mapping:
        cm = lifecycle_state(node, 'collision_monitor')
        checks.append(('Obstacle stop', cm == 'active', cm))
    node.destroy_node()
    rclpy.shutdown()

    for name, ok, detail in checks:
        print(f'{name + ":":<15} {"OK" if ok else "PROBLEM"}  ({detail})')
    if last_scan:
        print(lidar_view(last_scan[0]))
    bad = [name for name, ok, _ in checks if not ok]
    if bad:
        print('Not ready: ' + ', '.join(bad) + '. Try: ./start.sh stop; ./start.sh')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
