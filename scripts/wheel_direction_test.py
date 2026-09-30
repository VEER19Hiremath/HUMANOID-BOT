#!/usr/bin/env python3
"""Drive ONE wheel at a time with its direction line at the 'forward' level.

Stack stopped (./start.sh stop). Watch each wheel: it must roll FORWARD.
The "forward" level of each wheel comes from LEFT_INVERTED / RIGHT_INVERTED
in arduino/wheelodom.ino (inverted: F/R LOW = forward, else HIGH). A wheel
that rolls backward needs its *_INVERTED flipped (and the Mega reflashed).

Usage: python3 scripts/wheel_direction_test.py [L] [R]   (default both)
"""
import glob
import re
from pathlib import Path
import sys
import time

import serial

INO = (Path(__file__).resolve().parent.parent / 'arduino/wheelodom.ino').read_text()


def forward_level(side):
    inverted = re.search(rf'#define {side}_INVERTED\s+(true|false)', INO).group(1) == 'true'
    return 0 if inverted else 1


port = (glob.glob('/dev/serial/by-id/usb-Arduino*') or ['/dev/ttyACM0'])[0]
# A normal open resets the Mega into its safe start (motors off).
s = serial.Serial(port, 115200, timeout=0.1, write_timeout=1)
time.sleep(3.0)


last_counts = '?'


def pump():
    """Read everything the Mega sent: it must never fill up and stall."""
    global last_counts
    for line in s.read(8000).decode(errors='ignore').splitlines():
        if line.startswith('L:'):
            last_counts = line


def hold(cmd, secs):
    """Repeat the bench command every 0.5 s (the Mega drops it after 1.5 s)."""
    end = time.time() + secs
    while time.time() < end:
        s.write((cmd + '\n').encode())
        t = time.time()
        while time.time() - t < 0.5:
            pump()
            time.sleep(0.05)
    for _ in range(5):
        s.write(b'PINS OFF\n')
        pump()
        time.sleep(0.1)


sides = [a.upper() for a in sys.argv[1:]] or ['L', 'R']
for side in sides:
    name = 'LEFT' if side == 'L' else 'RIGHT'
    level = forward_level('LEFT' if side == 'L' else 'RIGHT')
    print(f'{name} wheel only, direction line {"HIGH" if level else "LOW"} (= forward), '
          'effort 15, 3 s ...', flush=True)
    pump()
    c0 = last_counts
    hold(f'PINS {side} B0 E1 F{level} A15', 3.0)
    print(f'  pulses {c0} -> {last_counts}', flush=True)
    time.sleep(3.0)
s.write(b'PINS OFF\n')
s.close()
print('done')
