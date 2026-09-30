#!/usr/bin/env python3
"""Bench drive test: run both hubs slowly, report speed pulses per wheel.

WHEELS MUST BE OFF THE FLOOR. Motor battery ON. Stop the stack first
(./start.sh stop). Uses the normal "VL:/VR:" protocol, so every firmware
safety (soft PWM cap, ramp, stall cut, 300 ms command timeout) stays active;
stopping this script stops the motors within 300 ms.

PASS = drivers reached PWM, no STALL, and both wheels gave speed pulses
(bench 2026-09-30: ≈ 26–29 pulses/s per lifted wheel at 0.06 m/s).

Usage: scripts/drive_test.py [port] [seconds] [speed_mps]
"""

import glob
import sys
import time

import serial

_MEGA = sorted(glob.glob('/dev/serial/by-id/usb-Arduino*'))
PORT = (sys.argv[1] if len(sys.argv) > 1 else '') or (
    _MEGA[0] if _MEGA else '/dev/ttyACM0')
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
SPEED = float(sys.argv[3]) if len(sys.argv) > 3 else 0.06


def parse(line):
    """'L:<n> R:<n>' -> (n, n), else None."""
    if not line.startswith('L:') or ' R:' not in line:
        return None
    left, right = line[2:].split(' R:', 1)
    try:
        return int(left), int(right)
    except ValueError:
        return None


def main():
    link = serial.Serial(PORT, 115200, timeout=0.05)
    time.sleep(2.0)  # Mega resets on open
    link.reset_input_buffer()
    cmd = f'VL:{SPEED:.3f} VR:{SPEED:.3f}\n'.encode()

    first = last = None
    events, status = [], []
    end = time.monotonic() + SECONDS
    next_send = 0.0
    try:
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_send:
                link.write(cmd)
                next_send = now + 0.1
            raw = link.readline().decode(errors='ignore').strip()
            counts = parse(raw)
            if counts is not None:
                first = first or counts
                last = counts
            elif raw.startswith('S:'):
                status.append(raw)
            elif raw and ('STALL' in raw or raw.startswith('ENC')):
                events.append(f'{SECONDS - (end - now):4.1f}s {raw}')
    finally:
        link.write(b'VL:0 VR:0\n')
        time.sleep(0.5)
        link.close()

    if first is None:
        print('FAIL: no counts — wrong port or firmware not running.')
        return 2
    dl, dr = last[0] - first[0], last[1] - first[1]
    peak = [s for s in status if 'pwmL=0 ' not in s]
    print(f'Drove {SECONDS:.0f}s at {SPEED} m/s')
    print(f'  PWM reached : {peak[-1] if peak else "never (driver not driven)"}')
    print(f'  Counts      : L={dl}  R={dr}')
    for line in events:
        print(f'  Mega        : {line}')
    stalled = any('STALL' in e for e in events)
    ok = bool(peak) and not stalled and dl > 0 and dr > 0
    if not ok and (dl == 0 or dr == 0):
        print('  Note        : a wheel gave no speed pulses — check its F/R '
              '(yellow) / BRK (blue) pins, driver LED, and D2/D3 feedback wire.')
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
