#!/usr/bin/env python3
"""Pick room goals by clicking the map in RViz, then save them.

Run while the map is shown in RViz (during ./start.sh map, or ./start.sh).
For each name below, use RViz's "Publish Point" tool and click that spot on
the map. Press Enter in this terminal to skip a room (keeps it out).

Writes:
  maps/floor_map.yaml (or --map)  "# room: name x y" lines: the voice node and
                                  RViz room markers read the rooms from here
  src/hospital_delivery/hospital_delivery/room_config.py  ROOM_COORDS (fallback)
  src/hospital_bringup/config/nav2_params.yaml            amcl initial_pose = home

Usage: scripts/pick_rooms.py [--map maps/x.yaml] [--dry-run]
"""

import re
import select
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node

ROOT = Path(__file__).resolve().parent.parent
ROOM_CONFIG = ROOT / 'src/hospital_delivery/hospital_delivery/room_config.py'
NAV2_PARAMS = ROOT / 'src/hospital_bringup/config/nav2_params.yaml'
NAMES = ['home', 'room1', 'room2', 'room3', 'room4', 'room5']


class Picker(Node):
    def __init__(self):
        super().__init__('room_picker')
        self.point = None
        self.create_subscription(
            PointStamped, '/clicked_point', self._clicked, 10)

    def _clicked(self, msg):
        self.point = (round(msg.point.x, 2), round(msg.point.y, 2))


def wait_click(node, name):
    """Return (x, y) for the next click, or None if Enter is pressed."""
    print(f'Click {name} with Publish Point (Enter = skip) ...', flush=True)
    node.point = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.point is not None:
            return node.point
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            return None
    return None


def write_rooms(rooms):
    text = ROOM_CONFIG.read_text()
    body = ''.join(
        f'    "{name}": ({x:.2f}, {y:.2f}),\n' for name, (x, y) in rooms.items())
    block = ('ROOM_COORDS = {\n'
             '    # Picked in RViz with scripts/pick_rooms.py.\n'
             f'{body}}}')
    new, count = re.subn(r'ROOM_COORDS = \{.*?\n\}', block, text, flags=re.S)
    if count != 1:
        raise RuntimeError('ROOM_COORDS block not found in room_config.py')
    ROOM_CONFIG.write_text(new)


def write_map_rooms(map_yaml, rooms):
    """Replace the map's "# room:" lines (kept just before 'image:')."""
    lines = [ln for ln in map_yaml.read_text().splitlines(keepends=True)
             if not ln.startswith('# room:')]
    at = next((i for i, ln in enumerate(lines) if ln.startswith('image:')), 0)
    lines[at:at] = [f'# room: {n} {x:.2f} {y:.2f}\n' for n, (x, y) in rooms.items()]
    map_yaml.write_text(''.join(lines))


def write_home(x, y):
    text = NAV2_PARAMS.read_text()
    new, count = re.subn(
        r'(\n    initial_pose:\n      x: )[-\d.]+(\n      y: )[-\d.]+',
        rf'\g<1>{x}\g<2>{y}', text)
    if count != 1:
        raise RuntimeError('amcl initial_pose not found in nav2_params.yaml')
    NAV2_PARAMS.write_text(new)


def main():
    dry = '--dry-run' in sys.argv
    map_yaml = ROOT / 'maps/floor_map.yaml'
    if '--map' in sys.argv:
        map_yaml = Path(sys.argv[sys.argv.index('--map') + 1]).resolve()
    rclpy.init()
    node = Picker()
    rooms = {}
    try:
        for name in NAMES:
            point = wait_click(node, name)
            if point is None:
                print(f'  {name}: skipped')
                continue
            rooms[name] = point
            print(f'  {name}: {point}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if 'home' not in rooms:
        print('home is required (the robot starts there). Nothing saved.')
        return 1
    print('\nROOM_COORDS =', rooms)
    if dry:
        print('--dry-run: nothing written.')
        return 0
    if map_yaml.exists():
        write_map_rooms(map_yaml, rooms)
        print(f'Saved rooms to {map_yaml.relative_to(ROOT)}.')
    write_rooms(rooms)
    write_home(*rooms['home'])
    print(f'Saved rooms to {ROOM_CONFIG.relative_to(ROOT)} and AMCL home '
          f'{rooms["home"]} to {NAV2_PARAMS.relative_to(ROOT)}.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
