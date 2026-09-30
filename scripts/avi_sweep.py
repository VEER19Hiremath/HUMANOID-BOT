#!/usr/bin/env python3
"""Measure wheel speed vs the raw AVI byte (calibrates the AVI curve).

WHEELS LIFTED, battery ON, stack stopped. Drives both wheels forward with
raw AVI values via the firmware "PINS ... W<raw>" bench command and records
pulses/s per wheel. Bench 2026-09-30: speed FELL as AVI rose (22 -> ~35/s,
30 -> ~22/s), so the sweep starts at the known 30 and only goes up; it aborts
if a wheel exceeds ABORT_RATE, or if a higher byte ever spins faster than the slowest seen.

Usage: scripts/avi_sweep.py [port]
"""

import glob
import sys
import time

import serial

_MEGA = sorted(glob.glob('/dev/serial/by-id/usb-Arduino*'))
PORT = (sys.argv[1] if len(sys.argv) > 1 else '') or (
    _MEGA[0] if _MEGA else '/dev/ttyACM0')
RAWS = [30, 36, 44, 56, 72, 96, 128, 160, 200, 240, 255]
HOLD_S = 2.5
ABORT_RATE = 120.0   # wheels lifted: only an absurd speed aborts outright
L_FWD, R_FWD = 0, 1  # raw F/R level for forward (LEFT_INVERTED / RIGHT_INVERTED)


def counts(raw):
    if raw.startswith('L:') and ' R:' in raw:
        left, right = raw[2:].split(' R:', 1)
        try:
            return int(left), int(right)
        except ValueError:
            return None
    return None


def hold(link, raw):
    """Drive at a raw AVI for HOLD_S; return (left/s, right/s) after settling."""
    link.write(f'PINS L B0 E1 F{L_FWD} W{raw}\n'.encode())
    link.write(f'PINS R B0 E1 F{R_FWD} W{raw}\n'.encode())
    start = time.monotonic()
    first = last = None
    while time.monotonic() - start < HOLD_S:
        c = counts(link.readline().decode(errors='ignore').strip())
        if c is None:
            continue
        if time.monotonic() - start >= 1.0:   # ignore the first second (spin-up)
            first = first or (c, time.monotonic())
            last = (c, time.monotonic())
            dt = last[1] - first[1]
            if dt > 0.4:
                rl = (last[0][0] - first[0][0]) / dt
                rr = (last[0][1] - first[0][1]) / dt
                if max(rl, rr) > ABORT_RATE:
                    return rl, rr
    if first is None or last[1] <= first[1]:
        return 0.0, 0.0
    dt = last[1] - first[1]
    return ((last[0][0] - first[0][0]) / dt, (last[0][1] - first[0][1]) / dt)


def main():
    link = serial.Serial(PORT, 115200, timeout=0.05)
    time.sleep(2.0)  # Mega resets on open
    link.reset_input_buffer()
    rows = []
    try:
        for raw in RAWS:
            rl, rr = hold(link, raw)
            rows.append((raw, rl, rr))
            print(f'  AVI {raw:>3}: left {rl:5.1f}/s   right {rr:5.1f}/s', flush=True)
            slowest = min((max(r[1], r[2]) for r in rows[:-1]), default=None)
            if slowest is not None and max(rl, rr) > slowest * 1.3 + 3:
                print('ABORT: speed rose with a higher byte — the AVI curve is '
                      'not monotonic here.')
                break
            if max(rl, rr) > ABORT_RATE:
                print(f'ABORT: faster than {ABORT_RATE}/s — AVI is not inverted '
                      'above this point.')
                break
    finally:
        link.write(b'PINS OFF\n')
        time.sleep(0.3)
        link.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
