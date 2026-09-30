#!/usr/bin/env python3
"""Plan (don't drive) between every pair of rooms and check each path.

For each path: length, reverse legs, tightest turn radius, closest approach
of the robot body to a drawn wall, and whether it leaves the area bounds.
Needs the stack running (./start.sh). Wheels never move.

Usage: python3 scripts/plan_check.py [--map maps/x.yaml] [room ...]
(default map: the one start.sh picks, i.e. the newest of floor_map,
open_floor, area_30x40 that exists)
"""

import math
import sys
from itertools import permutations
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'src/hospital_delivery'))
from hospital_delivery.room_config import ROOM_COORDS, goal_yaw, rooms_from_map  # noqa: E402

MAP = next(ROOT / f'maps/{n}.yaml' for n in ('floor_map', 'open_floor', 'area_30x40')
           if (ROOT / f'maps/{n}.yaml').exists())
HALF_LEN, HALF_WID = 0.31, 0.23   # body 2 ft x 1.5 ft, as the costmap footprint


def load_map():
    meta = {}
    for line in MAP.read_text().splitlines():
        if ':' in line and not line.startswith('#'):
            k, v = line.split(':', 1)
            meta[k.strip()] = v.strip()
    data = (MAP.parent / meta['image']).read_bytes()
    # P5 header: magic, (comments), size, maxval
    parts, pos = [], 0
    while len(parts) < 4:
        end = data.index(b'\n', pos)
        line = data[pos:end]
        pos = end + 1
        if not line.startswith(b'#'):
            parts += line.split()
    w, h = int(parts[1]), int(parts[2])
    pix = data[pos:pos + w * h]
    res = float(meta['resolution'])
    ox, oy = (float(v) for v in meta['origin'].strip('[]').split(',')[:2])
    walls = [(ox + (c + 0.5) * res, oy + (h - 1 - r + 0.5) * res)
             for r in range(h) for c in range(w) if pix[r * w + c] < 50]
    bounds = [float(v) for v in next(
        line for line in MAP.read_text().splitlines()
        if line.startswith('# bounds:')).split(':')[1].split()]
    return walls, bounds


def body_clearance(x, y, yaw, walls):
    """Distance from the rectangular body to the nearest wall cell."""
    c, s = math.cos(yaw), math.sin(yaw)
    best = 99.0
    for wx, wy in walls:
        dx, dy = wx - x, wy - y
        if abs(dx) > 1.5 or abs(dy) > 1.5:
            continue
        lx, ly = c * dx + s * dy, -s * dx + c * dy   # wall in body frame
        ex = max(abs(lx) - HALF_LEN, 0.0)
        ey = max(abs(ly) - HALF_WID, 0.0)
        best = min(best, math.hypot(ex, ey))
    return best


def analyse(path, walls, bounds):
    pts = [(p.pose.position.x, p.pose.position.y,
            2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w))
           for p in path.poses]
    length, cusps, min_r = 0.0, 0, 99.0
    prev_dir = None
    for i in range(1, len(pts)):
        dx, dy = pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]
        step = math.hypot(dx, dy)
        length += step
        if step < 1e-4:
            continue
        heading = pts[i - 1][2]
        d = 1 if dx * math.cos(heading) + dy * math.sin(heading) >= 0 else -1
        if prev_dir is not None and d != prev_dir:
            cusps += 1
        prev_dir = d
        dyaw = abs(math.atan2(math.sin(pts[i][2] - pts[i - 1][2]),
                              math.cos(pts[i][2] - pts[i - 1][2])))
        if dyaw > 1e-3:
            min_r = min(min_r, step / dyaw)
    clear = min(body_clearance(x, y, t, walls) for x, y, t in pts[::2])
    xmin, xmax, ymin, ymax = bounds
    outside = sum(not (xmin <= x <= xmax and ymin <= y <= ymax) for x, y, _ in pts)
    return length, cusps, min_r, clear, outside


def main():
    global MAP
    args = sys.argv[1:]
    if '--map' in args:
        i = args.index('--map')
        MAP = Path(args[i + 1]).resolve()
        del args[i:i + 2]
    coords = rooms_from_map(MAP) or ROOM_COORDS
    rooms = args or list(coords)
    walls, bounds = load_map()
    print(f'map: {MAP.name}')
    rclpy.init()
    node = Node('plan_check')
    client = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
    if not client.wait_for_server(timeout_sec=30):
        print('planner not available: is ./start.sh running?')
        return 1
    bad = 0
    print(f'{"from":<6} {"to":<6} {"length":>7} {"reverses":>8} {"min turn r":>10} '
          f'{"wall gap":>8} {"outside":>7}')
    for a, b in permutations(rooms, 2):
        (ax, ay), (bx, by) = coords[a], coords[b]
        goal = ComputePathToPose.Goal()
        goal.use_start = True
        # Worst case start: the robot reached room a driving straight from b,
        # so it faces away from b and must turn round. Goal: straight in.
        for pose, (x, y), yaw in ((goal.start, (ax, ay), goal_yaw((bx, by), (ax, ay))),
                                  (goal.goal, (bx, by), goal_yaw((ax, ay), (bx, by)))):
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
        if a == 'home':
            goal.start.pose.orientation.z, goal.start.pose.orientation.w = 0.0, 1.0
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=15)
        handle = fut.result()
        if not handle or not handle.accepted:
            print(f'{a:<6} {b:<6} goal not accepted')
            bad += 1
            continue
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(node, res_fut, timeout_sec=20)
        res = res_fut.result()
        if not res or not res.result.path.poses:
            print(f'{a:<6} {b:<6} NO PATH')
            bad += 1
            continue
        length, cusps, min_r, clear, outside = analyse(res.result.path, walls, bounds)
        flag = '' if clear >= 0.05 and outside == 0 else '  <-- check'
        bad += bool(flag)
        print(f'{a:<6} {b:<6} {length:6.1f}m {cusps:8d} {min_r:9.2f}m {clear:7.2f}m '
              f'{outside:7d}{flag}', flush=True)
    node.destroy_node()
    rclpy.shutdown()
    print(f'\n{bad} problem path(s)')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
