#!/usr/bin/env python3
"""Per-wheel direction check: each hub must run forward AND reverse.

WHEELS LIFTED, battery ON, stack stopped. Drives forward then reverse for
SECONDS each through the normal firmware path and reports, per wheel, which
directions produced speed pulses. A correctly phased hub runs BOTH ways.
A hub that runs one way only sits stalled under PWM in its dead direction,
which is what blows fuses — keep SECONDS short. Usual cause: F/R and BRK
wires on the wrong Mega pins (see README wiring table).

Usage: scripts/phase_check.py [port] [seconds]
"""

import glob
import sys
import time

import serial

_MEGA = sorted(glob.glob('/dev/serial/by-id/usb-Arduino*'))
PORT = (sys.argv[1] if len(sys.argv) > 1 else '') or (
    _MEGA[0] if _MEGA else '/dev/ttyACM0')
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 1.5
SPEED = 0.06
MOVED = 10  # pulses in one run that count as "ran"


def drive(link, speed):
    """Command speed for SECONDS; return (dL, dR) pulses or None."""
    first = last = None
    end = time.monotonic() + SECONDS
    next_send = 0.0
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= next_send:
            link.write(f'VL:{speed:.3f} VR:{speed:.3f}\n'.encode())
            next_send = now + 0.1
        raw = link.readline().decode(errors='ignore').strip()
        if raw.startswith('L:') and ' R:' in raw:
            left, right = raw[2:].split(' R:', 1)
            last = (int(left), int(right))
            first = first or last
    link.write(b'VL:0 VR:0\n')
    time.sleep(1.0)  # coast down before the next direction
    link.reset_input_buffer()
    if first is None:
        return None
    return last[0] - first[0], last[1] - first[1]


def main():
    link = serial.Serial(PORT, 115200, timeout=0.05)
    time.sleep(2.0)  # Mega resets on open
    link.reset_input_buffer()
    try:
        fwd = drive(link, SPEED)
        rev = drive(link, -SPEED)
    finally:
        link.write(b'VL:0 VR:0\n')
        link.close()
    if fwd is None or rev is None:
        print('FAIL: no counts — wrong port or firmware not running.')
        return 2

    ok = True
    for i, name in enumerate(('LEFT', 'RIGHT')):
        f, r = fwd[i] >= MOVED, rev[i] >= MOVED
        if f and r:
            verdict = 'OK — runs both ways'
        elif f or r:
            verdict = (f'ONE-WAY — runs {"forward" if f else "reverse"} '
                       'only; check F/R (yellow) and BRK (blue) pins')
            ok = False
        else:
            verdict = 'DEAD — check fuse, power, Hall plug'
            ok = False
        print(f'  {name:<5} fwd={fwd[i]:>4}  rev={rev[i]:>4}  {verdict}')
    print('PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
