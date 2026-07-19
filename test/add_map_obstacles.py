#!/usr/bin/env python3
# Copyright 2026 emcl2_ros2 developers
# SPDX-FileCopyrightText: 2026 emcl2_ros2 developers
# SPDX-License-Identifier: LGPL-3.0-or-later
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""
Draw extra occupied rectangles onto an occupancy map (map/world mismatch eval).

Reads a base map (yaml + binary P5 pgm), paints axis-aligned obstacle rectangles
given in world metres, and writes mismatch_map.{pgm,yaml} into an output
directory with the same resolution and origin. Used to give emcl2 a map that
disagrees with the simulated world -- here the map gains obstacles the world
does not have (the "phantom obstacle" / removed-from-world case). No image
library is needed; the P5 pgm is parsed directly.

Usage:
  add_map_obstacles.py <base.yaml> <out_dir> "x,y,sx,sy;x,y,sx,sy;..."
where each obstacle is a centre (x, y) and footprint (sx, sy) in metres.
"""

import os
import sys


def read_pgm(path):
    """Parse a binary (P5) pgm into (width, height, maxval, bytearray)."""
    with open(path, 'rb') as f:
        raw = f.read()
    pos = 0

    def token():
        nonlocal pos
        while pos < len(raw) and raw[pos:pos + 1].isspace():
            pos += 1
        if pos < len(raw) and raw[pos:pos + 1] == b'#':      # comment to end of line
            while pos < len(raw) and raw[pos:pos + 1] != b'\n':
                pos += 1
            return token()
        start = pos
        while pos < len(raw) and not raw[pos:pos + 1].isspace():
            pos += 1
        return raw[start:pos]

    magic = token()
    if magic != b'P5':
        raise ValueError('not a binary (P5) pgm: {}'.format(path))
    width = int(token())
    height = int(token())
    maxval = int(token())
    pos += 1  # exactly one whitespace byte separates the header from the data
    return width, height, maxval, bytearray(raw[pos:pos + width * height])


def read_yaml(path):
    """Minimal reader for the resolution/origin/image fields of a map yaml."""
    meta = {}
    for line in open(path):
        line = line.split('#', 1)[0].strip()
        if ':' not in line:
            continue
        key, val = line.split(':', 1)
        meta[key.strip()] = val.strip()
    return meta


def main():
    if len(sys.argv) < 4:
        sys.stderr.write('usage: add_map_obstacles.py base.yaml out_dir "x,y,sx,sy;..."\n')
        return 2
    base_yaml, out_dir, spec = sys.argv[1], sys.argv[2], sys.argv[3]

    meta = read_yaml(base_yaml)
    res = float(meta['resolution'])
    origin = meta['origin'].strip('[]').split(',')
    ox, oy = float(origin[0]), float(origin[1])
    img_path = meta['image'].strip('"\'')
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(base_yaml)), img_path)

    width, height, maxval, px = read_pgm(img_path)

    painted = 0
    for obstacle in spec.split(';'):
        obstacle = obstacle.strip()
        if not obstacle:
            continue
        cx, cy, sx, sy = (float(v) for v in obstacle.split(','))
        col0 = int(round((cx - sx / 2 - ox) / res))
        col1 = int(round((cx + sx / 2 - ox) / res))
        row_lo = int(round((cy - sy / 2 - oy) / res))
        row_hi = int(round((cy + sy / 2 - oy) / res))
        for col in range(max(0, col0), min(width, col1 + 1)):
            for r in range(max(0, row_lo), min(height, row_hi + 1)):
                px[col + (height - 1 - r) * width] = 0  # occupied (black); row 0 = top
        painted += 1

    out_pgm = os.path.join(out_dir, 'mismatch_map.pgm')
    out_yaml = os.path.join(out_dir, 'mismatch_map.yaml')
    with open(out_pgm, 'wb') as f:
        f.write('P5\n{} {}\n{}\n'.format(width, height, maxval).encode())
        f.write(bytes(px))
    with open(out_yaml, 'w') as f:
        f.write('image: mismatch_map.pgm\n')
        f.write('resolution: {}\n'.format(meta['resolution']))
        f.write('origin: {}\n'.format(meta['origin']))
        f.write('negate: {}\n'.format(meta.get('negate', '0')))
        f.write('occupied_thresh: {}\n'.format(meta.get('occupied_thresh', '0.65')))
        f.write('free_thresh: {}\n'.format(meta.get('free_thresh', '0.196')))
    sys.stderr.write('[map] wrote {} with {} extra obstacle(s)\n'.format(out_yaml, painted))
    print(out_yaml)
    return 0


if __name__ == '__main__':
    sys.exit(main())
