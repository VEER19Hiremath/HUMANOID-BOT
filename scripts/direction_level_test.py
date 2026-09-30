#!/usr/bin/env python3
"""Which direction (F/R, yellow wire) level drives each wheel forward?

Stack stopped (./start.sh stop). Drives ONE wheel at a time, slowly, for
2.5 s with the brake line LOW, then HIGH. Each wheel must turn with exactly
one level: that is its release level (L_BRK_RELEASE / R_BRK_RELEASE in
arduino/wheelodom.ino). Bench mode stops by itself 1.5 s after the last
command, and this script reads the Mega's output so it can't stall.

Usage: python3 scripts/brake_level_test.py
"""
import glob
import time

import serial

port = (glob.glob('/dev/serial/by-id/usb-Arduino*') or ['/dev/ttyACM0'])[0]
s = serial.Serial(port, 115200, timeout=0.05, write_timeout=1)   # resets the Mega: safe start
time.sleep(3.0)
counts = {'L': 0, 'R': 0}


def pump():
    for line in s.read(8000).decode(errors='ignore').splitlines():
        if line.startswith('L:') and ' R:' in line:
            try:
                left, right = line[2:].split(' R:')
                counts['L'], counts['R'] = int(left), int(right)
            except ValueError:
                pass   # a line cut in half by the read


def run(cmd, secs):
    end = time.time() + secs
    while time.time() < end:
        s.write((cmd + '\n').encode())
        t = time.time()
        while time.time() - t < 0.4:
            pump()
            time.sleep(0.05)
    for _ in range(5):
        s.write(b'PINS OFF\n')
        pump()
        time.sleep(0.1)


steps = [(1, 'L', 0), (2, 'L', 1), (3, 'R', 0), (4, 'R', 1)]   # (step, wheel, F/R level)
for n, side, brk in steps:
    name = 'LEFT' if side == 'L' else 'RIGHT'
    print(f'STEP {n}: {name} wheel, DIRECTION line {"HIGH" if brk else "LOW"} ...', flush=True)
    pump()
    before = counts[side]
    run(f'PINS {side} B0 E1 F{brk} A15', 2.5)
    print(f'        pulses {counts[side] - before:+d}', flush=True)
    time.sleep(2.0)
s.write(b'PINS OFF\n')
s.close()
print('done')
