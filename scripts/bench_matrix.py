#!/usr/bin/env python3
"""Bench test matrix: every BRK / ENBL / F-R / AVI case per wheel, hands off.

WHEELS LIFTED, motor battery ON, stack stopped (./start.sh stop), nobody
touching the wheels. Each case holds raw pin levels for a few seconds via the
firmware "PINS" bench command (AVI capped at PWM_MAX), or drives through the
normal "VL:/VR:" path, and records the D2/D3 speed pulses per wheel.
A wheel "moved" when its pulse count rises.

Usage: scripts/bench_matrix.py [port] [seconds-per-case]
"""

import glob
import sys
import time

import serial

_MEGA = sorted(glob.glob('/dev/serial/by-id/usb-Arduino*'))
PORT = (sys.argv[1] if len(sys.argv) > 1 else '') or (
    _MEGA[0] if _MEGA else '/dev/ttyACM0')
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
MOVED = 3  # pulses in one case that count as "moved"

# Raw F/R level for "forward" (firmware LEFT_INVERTED=true, RIGHT_INVERTED=false).
L_FWD, R_FWD = 0, 1

CASES = [
    # (group, label, command or ('VL', vl, vr))
    ('Idle, both', 'BRK0 ENBL0 (firmware rest)', 'PINS B0 E0'),
    ('Idle, both', 'BRK0 ENBL1', 'PINS B0 E1'),
    ('Idle, both', 'BRK1 ENBL0', 'PINS B1 E0'),
    ('Idle, both', 'BRK1 ENBL1', 'PINS B1 E1'),
    ('Right only', 'fwd AVI15', f'PINS R B0 E1 F{R_FWD} A15'),
    ('Right only', 'fwd AVI22', f'PINS R B0 E1 F{R_FWD} A22'),
    ('Right only', 'fwd AVI28', f'PINS R B0 E1 F{R_FWD} A28'),
    ('Right only', 'fwd AVI32', f'PINS R B0 E1 F{R_FWD} A32'),
    ('Right only', 'rev AVI28', f'PINS R B0 E1 F{1 - R_FWD} A28'),
    ('Right only', 'fwd AVI28 BRK1 (brake inverted?)', f'PINS R B1 E1 F{R_FWD} A28'),
    ('Right only', 'fwd AVI28 ENBL0 (enable inverted?)', f'PINS R B0 E0 F{R_FWD} A28'),
    ('Left only', 'fwd AVI22', f'PINS L B0 E1 F{L_FWD} A22'),
    ('Left only', 'fwd AVI28', f'PINS L B0 E1 F{L_FWD} A28'),
    ('Left only', 'rev AVI28', f'PINS L B0 E1 F{1 - L_FWD} A28'),
    ('Firmware drive', 'VL/VR +0.06 (forward)', ('VL', 0.06, 0.06)),
    ('Firmware drive', 'VL/VR -0.06 (reverse)', ('VL', -0.06, -0.06)),
    ('Idle, both', 'BRK0 ENBL1 again (latch check)', 'PINS B0 E1'),
]


def parse(line):
    """'L:<n> R:<n>' -> (n, n), else None."""
    if not line.startswith('L:') or ' R:' not in line:
        return None
    left, right = line[2:].split(' R:', 1)
    try:
        return int(left), int(right)
    except ValueError:
        return None


def run_case(link, action):
    first = last = None
    notes = set()
    end = time.monotonic() + SECONDS
    next_send = 0.0
    if isinstance(action, str):
        link.write((action + '\n').encode())
    while time.monotonic() < end:
        now = time.monotonic()
        if not isinstance(action, str) and now >= next_send:
            link.write(f'VL:{action[1]:.3f} VR:{action[2]:.3f}\n'.encode())
            next_send = now + 0.1
        raw = link.readline().decode(errors='ignore').strip()
        counts = parse(raw)
        if counts is not None:
            first = first or counts
            last = counts
        elif raw and ('STALL' in raw or raw.startswith('ENC')
                      or 'timeout' in raw):
            notes.add(raw)
    link.write(b'PINS OFF\n' if isinstance(action, str) else b'VL:0 VR:0\n')
    time.sleep(0.6)  # let the hubs coast down before the next case
    link.reset_input_buffer()
    if first is None:
        return None
    return last[0] - first[0], last[1] - first[1], sorted(notes)


def main():
    link = serial.Serial(PORT, 115200, timeout=0.05)
    time.sleep(2.0)  # Mega resets on open
    link.reset_input_buffer()
    rows = []
    try:
        for group, label, action in CASES:
            print(f'… {group}: {label}', flush=True)
            result = run_case(link, action)
            if result is None:
                print('FAIL: no counts from Mega — wrong port or firmware.')
                return 2
            rows.append((group, label) + result)
    finally:
        link.write(b'PINS OFF\nVL:0 VR:0\n')
        time.sleep(0.2)
        link.close()

    def mark(n):
        return f'{n:>4} {"MOVED" if n >= MOVED else "-":<5}'

    print(f'\n{"Group":<15} {"Case":<36} {"Left":<10} {"Right":<10} Notes')
    for group, label, dl, dr, notes in rows:
        print(f'{group:<15} {label:<36} {mark(dl)} {mark(dr)} {" ".join(notes)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
