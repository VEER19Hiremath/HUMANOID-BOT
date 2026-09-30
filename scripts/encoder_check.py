#!/usr/bin/env python3
"""Hand-spin encoder check, one wheel at a time, with live counts.

Sends no drive commands, so the motors stay dead (firmware CMD_TIMEOUT).
Stop the stack first (./start.sh stop) — only one process can own the port.

Turn each wheel ONE full revolution when prompted. Expected ≈ COUNTS_PER_TURN
(wheel_radius / distance_per_pulse from base_controller). Verdicts:
  OK    count ≥ 50 % of expected
  WEAK  some counts but too few — loose blue signal wire, poor 5V/GND
  DEAD  zero — yellow → Mega 5V, black → GND, blue → D2 left / D3 right
  CROSS the other wheel's count moved instead — signal wires swapped

Usage: scripts/encoder_check.py [port] [seconds-per-wheel]
Exit 0 only when both wheels are OK.
"""

import glob
import math
import sys
import time

import serial

WHEEL_RADIUS_M = 0.0762
DISTANCE_PER_PULSE_M = 0.00531
COUNTS_PER_TURN = 2 * math.pi * WHEEL_RADIUS_M / DISTANCE_PER_PULSE_M

_MEGA = sorted(glob.glob('/dev/serial/by-id/usb-Arduino*'))
PORT = (sys.argv[1] if len(sys.argv) > 1 else '') or (
    _MEGA[0] if _MEGA else '/dev/ttyACM0')
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0


def parse(line):
    """'L:<n> R:<n>' -> (n, n), else None."""
    if not line.startswith('L:') or ' R:' not in line:
        return None
    left, right = line[2:].split(' R:', 1)
    try:
        return int(left), int(right)
    except ValueError:
        return None


def latest(link, timeout=1.0):
    """Most recent counts within timeout, or None."""
    counts = None
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        raw = link.readline().decode(errors='ignore').strip()
        parsed = parse(raw)
        if parsed is not None:
            counts = parsed
            if not link.in_waiting:
                break
    return counts


def measure(link, name, idx):
    """Prompt for one wheel; return (own_delta, other_delta)."""
    for n in (3, 2, 1):
        print(f'\r  Get ready to turn the {name} wheel... {n}', end='', flush=True)
        time.sleep(1.0)
    start = latest(link)
    if start is None:
        return None
    print(f'\r  TURN the {name} wheel one full revolution now ({SECONDS:.0f}s)     ')
    now = start
    end = time.monotonic() + SECONDS
    while time.monotonic() < end:
        counts = latest(link, 0.3) or now
        if counts != now:
            now = counts
        remaining = max(0.0, end - time.monotonic())
        print(f'\r    {remaining:4.1f}s left  L={now[0] - start[0]:>5}  '
              f'R={now[1] - start[1]:>5}', end='', flush=True)
    print()
    return now[idx] - start[idx], now[1 - idx] - start[1 - idx]


def verdict(own, other):
    if own >= 0.5 * COUNTS_PER_TURN:
        return 'OK'
    if own == 0 and other > 0:
        return 'CROSS'
    if own == 0:
        return 'DEAD'
    return 'WEAK'


def main():
    link = serial.Serial(PORT, 115200, timeout=0.2)
    time.sleep(2.0)  # Mega resets on open
    link.reset_input_buffer()
    if latest(link, 3.0) is None:
        print(f'FAIL: no "L:/R:" lines on {PORT} — wrong port, or Mega '
              'firmware not running.')
        return 2
    print(f'Encoder check on {PORT} — expect ≈{COUNTS_PER_TURN:.0f} '
          'counts per full wheel turn.\n')

    results = {}
    for name, idx in (('LEFT', 0), ('RIGHT', 1)):
        sample = measure(link, name, idx)
        if sample is None:
            print('FAIL: Mega stopped reporting counts.')
            return 2
        results[name] = (verdict(*sample), sample[0])
    link.close()

    print()
    hints = {
        'WEAK': 'reseat the blue signal wire and the 5V/GND jumpers',
        'DEAD': 'check yellow → 5V, black → GND, blue → D2 (left) / D3 (right)',
        'CROSS': 'blue signal wires are swapped between D2 and D3',
    }
    for name, (v, own) in results.items():
        pct = 100.0 * own / COUNTS_PER_TURN
        line = f'  {name:<5} {v:<5} {own:>4} counts ({pct:.0f}% of a turn)'
        print(line + (f' — {hints[v]}' if v in hints else ''))
    return 0 if all(v == 'OK' for v, _ in results.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
