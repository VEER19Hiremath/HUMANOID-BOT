#!/usr/bin/env python3
"""Floor tour: drive to every room from home and record what happened.

./start.sh running, robot at home facing +x, obstacle stop on. Sends each
room as a voice command (/voice_command, same path as speech) and records
per leg: Nav2 outcome, time, odometry pose, obstacle stops/slowdowns, Mega
stalls and errors. A leg with repeated stalls is stopped ("stop") and the
tour moves on. Results: run_logs/latest/floor_tour.json and a summary.

Usage: python3 scripts/floor_tour.py [room ...]   (default: every room of the map, then home)
"""

import json
import math
import sys
import time
from pathlib import Path

import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from e2e_test import LOGS, ROOT, Harness, phrase  # noqa: E402

sys.path.insert(0, str(ROOT / 'src/hospital_delivery'))
from hospital_delivery.room_config import rooms_from_map  # noqa: E402

LEG_TIMEOUT = 300.0
MAX_STALLS = 2
BOUND = 2.65   # m from home: 20 x 20 ft area minus a body margin


def main():
    rooms_all = rooms_from_map(ROOT / 'maps/open_floor.yaml')
    order = sys.argv[1:] or [r for r in rooms_all if r != 'home'] + ['home']
    order = [r if r == 'home' or r.startswith('room') else f'room{r}' for r in order]
    rooms = rooms_from_map(ROOT / 'maps/open_floor.yaml')
    rclpy.init()
    h = Harness()
    h.pump(2.0)
    nav_extra = []
    results = []
    for room in order:
        t0 = time.monotonic()
        h.say(phrase(room))
        # Re-send if no trip started (lost message): at most twice.
        for _ in range(2):
            if h.wait_for({'begin'}, 6, t0)[0]:
                break
            h.say(phrase(room))
        outcome, stalls, stops, slows = None, 0, 0, 0
        while time.monotonic() - t0 < LEG_TIMEOUT:
            h.pump(0.5)
            ev = [(t, k, x) for t, k, x in h.events if t >= t0]
            stalls = sum(k == 'stall' for _, k, _ in ev)
            ends = [k for _, k, _ in ev if k in ('success', 'failed', 'canceled')]
            if ends:
                outcome = ends[0]
                break
            p = h.pose()
            if p and (abs(p[0]) > BOUND or abs(p[1]) > BOUND):
                h.say('stop')
                h.pump(3.0)
                outcome = f'STOPPED at boundary {p[0]:+.2f},{p[1]:+.2f}'
                break
            if stalls >= MAX_STALLS:
                h.say('stop')
                h.pump(3.0)
                outcome = 'stopped after stalls'
                break
        else:
            h.say('stop')
            h.pump(3.0)
            outcome = 'timeout (stopped)'
        # Obstacle stop / slowdown events this leg, from nav.log.
        text = (LOGS / 'nav.log').read_text(errors='ignore').splitlines()
        stamp0 = time.time() - (time.monotonic() - t0)
        for line in text:
            if 'collision_monitor' not in line or 'Robot to' not in line:
                continue
            try:
                ts = float(line.split('[')[3].split(']')[0])
            except (IndexError, ValueError):
                continue
            if ts >= stamp0:
                stops += 'to stop' in line
                slows += 'slowdown' in line
        h.wait_still(3.0)
        p = h.pose()
        gx, gy = rooms[room]
        err = math.hypot(p[0] - gx, p[1] - gy) if p else None
        errors = [x for t, k, x in h.events if t >= t0 and k == 'error']
        res = dict(room=room, outcome=outcome, secs=round(time.monotonic() - t0, 1),
                   pose=[round(v, 2) for v in p] if p else None,
                   odom_error=None if err is None else round(err, 2),
                   obstacle_stops=stops, slowdowns=slows, stalls=stalls,
                   errors=errors[:3])
        results.append(res)
        print(f"{room:<6} {outcome:<22} {res['secs']:6.1f}s pose {res['pose']} "
              f"err {res['odom_error']}  stops {stops} slow {slows} stalls {stalls}"
              + (f"  ERR {errors[0][:80]}" if errors else ''), flush=True)
        (LOGS / 'floor_tour.json').write_text(json.dumps(results, indent=1))
        h.pump(3.0)
    h.destroy_node()
    rclpy.shutdown()
    ok = sum(r['outcome'] == 'success' for r in results)
    print(f'\n{ok}/{len(results)} legs reached their room (by odometry)')


if __name__ == '__main__':
    main()
