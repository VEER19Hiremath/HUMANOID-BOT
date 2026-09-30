#!/usr/bin/env python3
"""Draw a map with its rooms and the robot at home, as a PNG for the docs.

Reads the map yaml (image, resolution, origin, "# room:" lines) and draws:
the map, a 1 ft grid with feet labels, each room as the coloured zone RViz
shows, and the robot's 2 x 1.5 ft footprint at home facing +x (its start
heading). No ROS needed.

Usage: python3 scripts/render_map.py [maps/open_floor.yaml] [docs/room_map.png]
"""

import sys
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrow, Rectangle  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'src/hospital_delivery'))
from hospital_delivery.room_config import rooms_from_map  # noqa: E402

FT = 0.3048
BODY_L, BODY_W = 0.61, 0.46     # 2 ft x 1.5 ft
ZONE = 0.9                      # as room_markers.py
COLORS = [(0.20, 0.60, 1.00), (1.00, 0.55, 0.15), (0.30, 0.80, 0.40),
          (0.80, 0.35, 0.85), (0.95, 0.80, 0.20), (0.20, 0.80, 0.80)]


def read_map(yaml_path):
    meta = {}
    for line in yaml_path.read_text().splitlines():
        if ':' in line and not line.startswith('#'):
            k, v = line.split(':', 1)
            meta[k.strip()] = v.strip()
    data = (yaml_path.parent / meta['image']).read_bytes()
    parts, pos = [], 0
    while len(parts) < 4:              # P5 header: magic, size, maxval
        end = data.index(b'\n', pos)
        line = data[pos:end]
        pos = end + 1
        if not line.startswith(b'#'):
            parts += line.split()
    w, h = int(parts[1]), int(parts[2])
    pix = list(data[pos:pos + w * h])
    rows = [pix[r * w:(r + 1) * w] for r in range(h)]
    res = float(meta['resolution'])
    ox, oy = (float(v) for v in meta['origin'].strip('[]').split(',')[:2])
    return rows, w, h, res, ox, oy


def main():
    yaml_path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'maps/open_floor.yaml'
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT / 'docs/room_map.png'
    rows, w, h, res, ox, oy = read_map(yaml_path)
    rooms = rooms_from_map(yaml_path) or {}
    x0, x1, y0, y1 = ox, ox + w * res, oy, oy + h * res

    fig, ax = plt.subplots(figsize=(8, 8), dpi=120)
    ax.imshow(rows, cmap='gray', vmin=0, vmax=255, extent=(x0, x1, y0, y1), origin='upper')
    # 1 ft grid, labelled in feet from home.
    ft_lo, ft_hi = int(x0 / FT), int(x1 / FT) + 1
    for f in range(ft_lo, ft_hi):
        lw = 0.9 if f % 5 == 0 else 0.3
        ax.axvline(f * FT, color='0.75', lw=lw, zorder=1)
        ax.axhline(f * FT, color='0.75', lw=lw, zorder=1)
    ticks = [f * FT for f in range(ft_lo, ft_hi) if f % 2 == 0]
    ax.set_xticks(ticks, [f'{round(t / FT)}' for t in ticks])
    ax.set_yticks(ticks, [f'{round(t / FT)}' for t in ticks])
    ax.set_xlabel('feet from home, forward (+x) →')
    ax.set_ylabel('feet from home, left (+y) →')

    for i, (name, (x, y)) in enumerate(sorted(rooms.items())):
        color = (0.6, 0.6, 0.6) if name == 'home' else COLORS[i % len(COLORS)]
        ax.add_patch(Rectangle((x - ZONE / 2, y - ZONE / 2), ZONE, ZONE, color=color,
                               alpha=0.35, zorder=2))
        ax.plot(x, y, 'o', color=color, markersize=7, zorder=4)
        label = 'Home (start)' if name == 'home' else name.replace('room', 'Room ')
        ax.text(x, y + ZONE / 2 + 0.08, f'{label}\n({x:+.2f}, {y:+.2f}) m', ha='center',
                va='bottom', fontsize=9, zorder=5)
    # Robot at home facing +x.
    ax.add_patch(Rectangle((-BODY_L / 2, -BODY_W / 2), BODY_L, BODY_W, fill=False,
                           edgecolor='black', lw=1.5, zorder=3))
    ax.add_patch(FancyArrow(0, 0, 0.45, 0, width=0.04, color='black', zorder=4))
    title = yaml_path.read_text().splitlines()[0].lstrip('# ').split('(')[0].strip()
    ax.set_title(f'{title}: home, rooms and the robot (2 x 1.5 ft) at its start pose')
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect('equal')
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    print(f'Wrote {out.relative_to(ROOT)}')


if __name__ == '__main__':
    main()
