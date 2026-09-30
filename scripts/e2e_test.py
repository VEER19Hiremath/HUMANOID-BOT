#!/usr/bin/env python3
"""End-to-end robot test: voice command -> Nav2 -> base -> Mega -> wheels.

Wheels LIFTED (or a clear floor), ./start.sh running on a drawn map. Commands
go in on /voice_command, the same path as recognised speech, and every trial
is judged from what actually happened:

  trip      "go to room N": Nav2 reports success, the robot is within
            ARRIVE_TOL of the room, and the wheels are still within 2 s
  stop      "go to room N", then "stop" after 4-15 s: wheels still within
            1.5 s, goal cancelled, still 5 s later
  redirect  "go to room N", then "go to room M" mid-trip: arrives at M
  noise     phrases that must be ignored ("one", "toronto", "[unk] ..."):
            no goal starts and the wheels don't move

Each trial also fails on a Mega stall, a new ERROR in the logs, or a goal
that doesn't end. Results: run_logs/latest/e2e_results.json and a summary.

Usage: python3 scripts/e2e_test.py [trials=100] [--seed N]
"""

import json
import math
import random
import re
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

ROOT = Path(__file__).resolve().parent.parent
LOGS = ROOT / 'run_logs/latest'
sys.path.insert(0, str(ROOT / 'src/hospital_delivery'))
from hospital_delivery.room_config import rooms_from_map  # noqa: E402

ARRIVE_TOL = 0.20
TRIP_TIMEOUT = 240.0
STILL_V, STILL_W = 0.003, 0.01
WORDS = {'room1': 'one', 'room2': 'two', 'room3': 'three', 'room4': 'four',
         'room5': 'five', 'home': 'home'}
NOISE = ['one', 'two', 'the one', 'a one', 'toronto', '[unk] gotta toronto',
         '[unk] go to room three', 'five', 'home', 'three four']
ERROR_IGNORE = re.compile(
    r'Unable to start transition|extrapolation|Message Filter|missed its desired rate|'
    r'Error_code parameters|Failed to get "laser"|getTransform', re.I)


def phrase(room):
    return 'go home' if room == 'home' else f'go to room {WORDS[room]}'


class LogTail:
    def __init__(self, path):
        self.path = path
        self.pos = path.stat().st_size if path.exists() else 0

    def new(self):
        if not self.path.exists():
            return []
        with open(self.path, 'rb') as f:
            f.seek(self.pos)
            data = f.read()
            self.pos = f.tell()
        return data.decode(errors='ignore').splitlines()


class Harness(Node):
    def __init__(self):
        super().__init__('e2e_test')
        self.pub = self.create_publisher(String, 'voice_command', 10)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.v = self.w = 0.0
        self.last_odom = 0.0
        self.create_subscription(Odometry, '/odom', self._odom, 20)
        self.nav = LogTail(LOGS / 'nav.log')
        self.base = LogTail(LOGS / 'base.log')
        self.voice = LogTail(LOGS / 'voice.log')
        self.events = []      # (time, kind, text) from the logs

    def _odom(self, msg):
        self.v, self.w = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        self.last_odom = time.monotonic()

    def say(self, text):
        """Publish once the voice node listens (an early message is lost)."""
        end = time.monotonic() + 15
        while self.pub.get_subscription_count() == 0 and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.pub.publish(String(data=text))

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def still(self):
        return abs(self.v) < STILL_V and abs(self.w) < STILL_W

    def pump(self, secs):
        """Spin and collect log events for secs seconds."""
        end = time.monotonic() + secs
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            for line in self.nav.new():
                if 'bt_navigator' in line and re.search(
                        r'Begin navigating|Goal succeeded|Goal failed|Goal canceled|aborted', line):
                    kind = ('begin' if 'Begin' in line else 'success' if 'succeeded' in line
                            else 'canceled' if 'canceled' in line else 'failed')
                    self.events.append((now, kind, line.split(']: ')[-1]))
                if '[ERROR]' in line and not ERROR_IGNORE.search(line):
                    self.events.append((now, 'error', line[-160:]))
            for line in self.base.new():
                if 'STALL' in line:
                    self.events.append((now, 'stall', line[-120:]))
                if '[ERROR]' in line:
                    self.events.append((now, 'error', line[-160:]))
                if 'Mega restarted' in line or 'link lost' in line:
                    self.events.append((now, 'error', line[-120:]))
            for line in self.voice.new():
                if 'Arrived at' in line:
                    self.events.append((now, 'voice_arrived', line.split(']: ')[-1]))
                if 'Traceback' in line or '[ERROR]' in line:
                    self.events.append((now, 'error', line[-160:]))
            if now >= end:
                return

    def wait_for(self, kinds, timeout, since):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.pump(0.2)
            for t, k, text in self.events:
                if t >= since and k in kinds:
                    return k, t
        return None, None

    def wait_still(self, timeout, hold=1.0):
        """Seconds until the wheels are still for `hold` s, or None."""
        t0 = time.monotonic()
        still_since = None
        while time.monotonic() - t0 < timeout:
            self.pump(0.05)
            if self.still():
                still_since = still_since or time.monotonic()
                if time.monotonic() - still_since >= hold:
                    return still_since - t0
            else:
                still_since = None
        return None


def trial_errors(h, since):
    return [text for t, k, text in h.events if t >= since and k in ('stall', 'error')]


def run_trip(h, rooms, here, target):
    t0 = time.monotonic()
    h.say(phrase(target))
    began, _ = h.wait_for({'begin'}, 10, t0)
    if not began:
        return dict(ok=False, why='no goal started')
    kind, t_end = h.wait_for({'success', 'failed', 'canceled'}, TRIP_TIMEOUT, t0)
    if kind != 'success':
        return dict(ok=False, why=f'goal {kind or "never ended"}')
    stop_s = h.wait_still(3.0)
    p = h.pose()
    gx, gy = rooms[target]
    err = math.hypot(p[0] - gx, p[1] - gy) if p else 99
    # The voice node must get the result too (it says "Arrived").
    voice_ok = any(k == 'voice_arrived' and t >= t0 for t, k, _ in h.events)
    ok = err <= ARRIVE_TOL and stop_s is not None and stop_s <= 2.0 and voice_ok
    why = ('' if ok else f'pose error {err:.2f} m' if err > ARRIVE_TOL
           else 'wheels kept turning' if stop_s is None or stop_s > 2.0
           else 'voice node never heard the result')
    return dict(ok=ok, why=why, secs=round(t_end - t0, 1), err=round(err, 3),
                stop_s=None if stop_s is None else round(stop_s, 2))


def run_stop(h, rooms, here, target):
    t0 = time.monotonic()
    h.say(phrase(target))
    began, _ = h.wait_for({'begin'}, 10, t0)
    if not began:
        return dict(ok=False, why='no goal started')
    h.pump(random.uniform(4, 15))
    moving = not h.still()
    t_stop = time.monotonic()
    h.say('stop')
    stop_s = h.wait_still(5.0, hold=0.5)
    ended, _ = h.wait_for({'canceled', 'success', 'failed'}, 5, t0)
    h.pump(5.0)
    stayed = h.still()
    ok = stop_s is not None and stop_s <= 1.5 and ended is not None and stayed
    why = ('' if ok else 'wheels kept turning' if stop_s is None or stop_s > 1.5
           else 'goal not cancelled' if ended is None else 'moved again after stop')
    return dict(ok=ok, why=why, stop_s=None if stop_s is None else round(stop_s, 2),
                was_moving=moving, secs=round(time.monotonic() - t_stop, 1))


def run_redirect(h, rooms, here, target):
    first = random.choice([r for r in rooms if r not in (here, target)])
    t0 = time.monotonic()
    h.say(phrase(first))
    if h.wait_for({'begin'}, 10, t0)[0] is None:
        return dict(ok=False, why='no first goal')
    h.pump(random.uniform(5, 12))
    t1 = time.monotonic()
    h.say(phrase(target))
    res = None
    for _ in range(2):   # first may finish before the redirect lands
        kind, t_end = h.wait_for({'success', 'failed', 'canceled'}, TRIP_TIMEOUT, t1)
        if kind is None:
            break
        p = h.pose()
        gx, gy = rooms[target]
        err = math.hypot(p[0] - gx, p[1] - gy) if p else 99
        res = dict(kind=kind, err=err, secs=round(t_end - t0, 1))
        if kind == 'success' and err <= ARRIVE_TOL:
            break
        t1 = t_end + 0.01
    if res is None:
        return dict(ok=False, why='redirect never ended', first=first)
    stop_s = h.wait_still(3.0)
    ok = res['kind'] == 'success' and res['err'] <= ARRIVE_TOL and stop_s is not None
    return dict(ok=ok, why='' if ok else f"{res['kind']} err {res['err']:.2f}", first=first,
                secs=res['secs'], err=round(res['err'], 3))


def run_noise(h, rooms, here, target):
    text = random.choice(NOISE)
    t0 = time.monotonic()
    h.say(text)
    h.pump(6.0)
    began = [k for t, k, _ in h.events if t >= t0 and k == 'begin']
    ok = not began and h.still()
    return dict(ok=ok, why='' if ok else 'noise started a goal / moved', phrase=text)


def main():
    args = sys.argv[1:]
    seed = int(args[args.index('--seed') + 1]) if '--seed' in args else int(time.time())
    args = [a for i, a in enumerate(args) if a != '--seed' and (i == 0 or args[i - 1] != '--seed')]
    trials = int(args[0]) if args else 100
    random.seed(seed)
    map_yaml = next(ROOT / f'maps/{n}.yaml' for n in ('floor_map', 'open_floor', 'area_30x40')
                    if (ROOT / f'maps/{n}.yaml').exists())
    rooms = rooms_from_map(map_yaml)
    kinds = ['trip'] * 60 + ['stop'] * 15 + ['redirect'] * 15 + ['noise'] * 10
    plan = [random.choice(kinds) for _ in range(trials)]

    rclpy.init()
    h = Harness()
    h.pump(2.0)
    results = []
    here = 'home'
    out = LOGS / 'e2e_results.json'
    print(f'map {map_yaml.name}, {trials} trials, seed {seed}', flush=True)
    for i, kind in enumerate(plan, 1):
        target = random.choice([r for r in rooms if r != here])
        t0 = time.monotonic()
        fn = dict(trip=run_trip, stop=run_stop, redirect=run_redirect, noise=run_noise)[kind]
        res = fn(h, rooms, here, target)
        errs = trial_errors(h, t0)
        if errs:
            res['ok'] = False
            res['why'] = (res.get('why') or '') + f' | {errs[0]}'
        res.update(n=i, kind=kind, frm=here, to=target)
        results.append(res)
        if kind in ('trip', 'redirect') and res.get('err') is not None and res['err'] <= ARRIVE_TOL:
            here = target
        elif kind in ('stop',):
            here = None   # somewhere between rooms
        if here is None:
            p = h.pose()
            here = min(rooms, key=lambda r: math.dist(rooms[r], p)) if p else 'home'
            # 'here' is only the nearest room; the next goal starts from the real pose.
        print(f"{i:3d} {kind:<8} {res['frm'] or '-':>5}->{target:<5} "
              f"{'PASS' if res['ok'] else 'FAIL'}  "
              + ' '.join(f'{k}={v}' for k, v in res.items()
                         if k not in ('ok', 'n', 'kind', 'frm', 'to') and v not in ('', None)),
              flush=True)
        out.write_text(json.dumps(results, indent=1))
        h.pump(2.0)
    h.destroy_node()
    rclpy.shutdown()

    passed = sum(r['ok'] for r in results)
    print(f'\n{passed}/{len(results)} passed (seed {seed})')
    for kind in ('trip', 'stop', 'redirect', 'noise'):
        rs = [r for r in results if r['kind'] == kind]
        if rs:
            print(f'  {kind:<8} {sum(r["ok"] for r in rs)}/{len(rs)}')
    return 0 if passed == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
