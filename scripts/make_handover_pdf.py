#!/usr/bin/env python3
"""Build docs/HANDOVER.pdf from docs/HANDOVER.md.

The Mermaid flow diagram (rendered on GitHub, not in a PDF) is drawn with
Graphviz as docs/architecture.png; the Markdown becomes print-styled HTML,
which LibreOffice turns into the PDF.

Needs: graphviz (dot), libreoffice, Python 'markdown' (pip install markdown).
Usage: ~/venv/bin/python3 scripts/make_handover_pdf.py
"""

import re
import shutil
import subprocess
import tempfile
from pathlib import Path

import markdown
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
DOCS = ROOT / 'docs'

# The same flow as the Mermaid block in HANDOVER.md.
DOT = r'''
digraph G {
  rankdir=TB; bgcolor="white"; nodesep=0.3; ranksep=0.35;
  node [shape=box, style="rounded,filled", fillcolor="#eef3f7", color="#5a7a90",
        fontname="DejaVu Sans", fontsize=11];
  edge [color="#5a7a90", fontname="DejaVu Sans", fontsize=9];
  MIC  [label="Headset / USB mic"];
  HFP  [label="hfp_mic.py\n(Bluetooth bridge)"];
  PW   [label="PipeWire\n(USB / wired mic)"];
  TXT  [label="/voice_command\n(typed commands)"];
  VOICE[label="voice_delivery_node\n(speech -> room)", fillcolor="#dcebf5"];
  NAV  [label="Nav2\nplanner + controller", fillcolor="#dcebf5"];
  LIDAR[label="RPLIDAR A2M8"];
  CM   [label="collision_monitor\n(obstacle stop)"];
  BASE [label="base_controller\n(wheel speeds, odometry)", fillcolor="#dcebf5"];
  MEGA [label="Arduino Mega\nwheelodom.ino", fillcolor="#fbeedd"];
  DRV  [label="2 x BLDC-5015A\ndrivers", fillcolor="#fbeedd"];
  MOT  [label="2 x hub motors", fillcolor="#fbeedd"];
  RVIZ [label="RViz\n(map, rooms, path)"];
  MIC -> HFP; MIC -> PW; HFP -> VOICE; PW -> VOICE; TXT -> VOICE;
  VOICE -> NAV [label="NavigateToPose goal"];
  LIDAR -> NAV [label="/scan"]; LIDAR -> CM [label="/scan"];
  NAV -> CM [label="/cmd_vel_smoothed"];
  CM -> BASE [label="/cmd_vel"];
  BASE -> MEGA [label="USB serial\nVL/VR, K"];
  MEGA -> DRV [label="speed, F/R,\nENBL, BRK"];
  DRV -> MOT;
  DRV -> MEGA [label="speed pulses\nD2/D3", style=dashed];
  MEGA -> BASE [label="pulse counts", style=dashed];
  BASE -> NAV [label="/odom + TF", style=dashed];
  NAV -> RVIZ;
}
'''

CSS = '''
@page { size: A4; margin: 16mm 14mm 16mm 14mm; }
body { font-family: "DejaVu Sans", "Liberation Sans", Arial, sans-serif;
       font-size: 10pt; line-height: 1.45; color: #1d2329; }
h1 { font-size: 22pt; color: #234; border-bottom: 2px solid #5a7a90;
     padding-bottom: 4pt; margin-top: 0; }
h2 { font-size: 15pt; color: #234; border-bottom: 1px solid #c8d3db;
     padding-bottom: 2pt; margin-top: 18pt; page-break-before: always; }
h3 { font-size: 12pt; color: #2d4a5e; margin-top: 12pt; }
table { border-collapse: collapse; width: 100%; margin: 6pt 0 10pt 0;
        font-size: 9pt; }
th { background: #e6eef3; text-align: left; }
th, td { border: 1px solid #b9c7d1; padding: 3pt 5pt; vertical-align: top; }
code { font-family: "DejaVu Sans Mono", monospace; font-size: 8.5pt;
       background: #f1f4f6; }
pre { background: #f1f4f6; border: 1px solid #d5dde3; padding: 6pt;
      font-size: 8.5pt; white-space: pre-wrap; }
blockquote { border-left: 3px solid #d08a2e; margin: 6pt 0; padding: 2pt 8pt;
             background: #fbf3e8; }
img { max-width: 100%; }
.cover { text-align: center; margin-top: 40pt; }
'''


def main():
    md = (DOCS / 'HANDOVER.md').read_text()
    # 1. Diagram image for the Mermaid block.
    subprocess.run(['dot', '-Tpng', '-Gdpi=150', '-o', str(DOCS / 'architecture.png')],
                   input=DOT.encode(), check=True)
    # LibreOffice sizes pictures from their DPI: store one that makes each
    # picture fit the page (diagram 16 cm wide, map 12 cm).
    for name, cm in (('architecture.png', 16.0), ('room_map.png', 12.0)):
        img = Image.open(DOCS / name)
        dpi = img.width / (cm / 2.54)
        img.save(DOCS / name, dpi=(dpi, dpi))
    md = re.sub(r'```mermaid.*?```', '![How the parts connect](architecture.png)', md,
                flags=re.S)
    # Colour emoji in the wiring table don't print reliably: plain words.
    for emoji in ('🟣 ', '🟡 ', '🔴 ', '🔵 ', '⚫ '):
        md = md.replace(emoji, '')
    # The first heading stays on the cover page (no page break before it).
    html_body = markdown.markdown(md, extensions=['tables', 'fenced_code', 'toc', 'sane_lists'])
    # Title, contents and section 1 share the first pages: no break before
    # the first two h2 headings.
    for _ in range(2):
        html_body = html_body.replace('<h2 id=', '<h2 style="page-break-before: auto" data-x id=', 1)
    html_body = html_body.replace(' data-x', '')
    # LibreOffice ignores max-width: give each picture a page-fitting size
    # (width and height, in CSS pixels at 96 per inch).
    for name, alt, cm in (('architecture.png', 'How the parts connect', 13.0),
                          ('room_map.png', 'Map with home and rooms', 15.0)):
        w, h = Image.open(DOCS / name).size
        px_w = round(cm / 2.54 * 96)
        html_body = html_body.replace(
            f'<img alt="{alt}" src="{name}"',
            f'<img width="{px_w}" height="{round(px_w * h / w)}" alt="{alt}" src="{name}"')
    html = (f'<!doctype html><html><head><meta charset="utf-8">'
            f'<title>Hospital Delivery Robot - Handover</title><style>{CSS}</style>'
            f'</head><body>{html_body}</body></html>')
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for img in ('architecture.png', 'room_map.png'):
            shutil.copy(DOCS / img, tmp / img)
        (tmp / 'HANDOVER.html').write_text(html)
        # A private profile: a running or first-time LibreOffice otherwise
        # blocks the headless conversion.
        # Profile locale en-GB: LibreOffice's default paper is then A4 (the
        # environment's locale doesn't reach the headless converter).
        reg = tmp / 'lo_profile/user/registrymodifications.xcu'
        reg.parent.mkdir(parents=True, exist_ok=True)
        reg.write_text(
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            '<oor:items xmlns:oor="http://openoffice.org/2001/registry" '
            'xmlns:xs="http://www.w3.org/2001/XMLSchema" '
            'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">\n'
            '<item oor:path="/org.openoffice.Setup/L10N"><prop oor:name="ooSetupSystemLocale" '
            'oor:op="fuse"><value>en-GB</value></prop></item>\n'
            '<item oor:path="/org.openoffice.Setup/L10N"><prop oor:name="ooLocale" '
            'oor:op="fuse"><value>en-GB</value></prop></item>\n'
            '</oor:items>\n')
        # Open as a Writer document (the Writer/Web PDF export hangs).
        subprocess.run(['libreoffice', f'-env:UserInstallation=file://{tmp}/lo_profile',
                        '--headless', '--infilter=HTML (StarWriter)',
                        '--convert-to', 'pdf:writer_pdf_Export', '--outdir', str(tmp),
                        str(tmp / 'HANDOVER.html')],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       timeout=180)
        shutil.copy(tmp / 'HANDOVER.pdf', DOCS / 'HANDOVER.pdf')
    print(f'Wrote {(DOCS / "HANDOVER.pdf").relative_to(ROOT)} and docs/architecture.png')


if __name__ == '__main__':
    main()
