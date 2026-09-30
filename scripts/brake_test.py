#!/usr/bin/env python3
"""Find which BRK / ENBL level actually brakes the hubs.

Uses the firmware bench command "PINS B<0|1> E<0|1>": raw pin levels with
AVI held at 0, so the motors are never driven. Keep turning the wheels by
hand the whole time; the phase with the most encoder counts is the one where
the hubs spin freely. Motor battery must be ON (an unpowered driver cannot
brake), wheels lifted off the floor. Stop the stack first (./start.sh stop).

Usage: scripts/brake_test.py [port] [seconds-per-phase]
"""

import glob
import sys
import time

import serial

_MEGA = sorted(glob.glob('/dev/serial/by-id/usb-Arduino*'))
PORT = (sys.argv[1] if len(sys.argv) > 1 else '') or (
    _MEGA[0] if _MEGA else '/dev/ttyACM0')
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
PHASES = ((0, 0), (1, 0), (0, 1), (1, 1))  # (BRK, ENBL) raw Mega levels


def parse(line):
    """'L:<n> R:<n>' -> (n, n), else None."""
    if not line.startswith('L:') or ' R:' not in line:
        return None
    left, right = line[2:].split(' R:', 1)
    try:
        return int(left), int(right)
    except ValueError:
        return None


def phase(link, brk, enbl):
    """Hold the levels for SECONDS; return (dL, dR, last status line)."""
    cmd = f'PINS B{brk} E{enbl}\n'.encode()
    link.write(cmd)
    first = last = None
    status = ''
    end = time.monotonic() + SECONDS
    while time.monotonic() < end:
        raw = link.readline().decode(errors='ignore').strip()
        if raw.startswith('S:'):
            status = raw
        counts = parse(raw)
        if counts is None:
            continue
        first = first or counts
        last = counts
    if first is None:
        return None
    return last[0] - first[0], last[1] - first[1], status


def main():
    link = serial.Serial(PORT, 115200, timeout=0.2)
    time.sleep(2.0)  # Mega resets on open
    link.reset_input_buffer()
    rows = []
    try:
        for brk, enbl in PHASES:
            print(f'Phase BRK={brk} ENBL={enbl} — keep turning the wheels '
                  f'({SECONDS:.0f}s)', flush=True)
            result = phase(link, brk, enbl)
            if result is None:
                print('FAIL: no counts — wrong port or old firmware.')
                return 2
            rows.append((brk, enbl) + result)
    finally:
        link.write(b'PINS OFF\n')
        time.sleep(0.2)
        link.close()

    print('\n  BRK ENBL   dL   dR  status')
    for brk, enbl, dl, dr, status in rows:
        print(f'   {brk}    {enbl}  {dl:>4} {dr:>4}  {status}')
    best = max(rows, key=lambda r: r[2] + r[3])
    if best[2] + best[3] == 0:
        print('\nNo counts in any phase — encoders are not reporting, so the '
              'brake level cannot be judged from counts. Report which phase '
              'felt free by hand.')
        return 1
    print(f'\nFreest phase: BRK={best[0]} ENBL={best[1]} '
          '(confirm it also felt free by hand).')
    return 0


if __name__ == '__main__':
    sys.exit(main())
