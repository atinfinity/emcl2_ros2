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
Plot ground-truth and estimated trajectories over the occupancy map.

Draws the occupancy grid (yaml + binary P5 pgm) as a grey background in world
coordinates, the ground truth as a dashed line, and the estimate as points
coloured by translation error -- so the APE can be read against the actual walls
and obstacles. Complements evo's plain xy plot, which has no map. No image
library is needed; the pgm is parsed directly.

Usage: plot_trajectory_on_map.py <gt.tum> <est.tum> <map.yaml> <out.png> [title]
"""

import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def read_yaml(path):
    meta = {}
    for line in open(path):
        line = line.split('#', 1)[0].strip()
        if ':' in line:
            key, val = line.split(':', 1)
            meta[key.strip()] = val.strip()
    return meta


def read_pgm(path):
    """Parse a binary (P5) pgm into a (height, width) uint8 numpy array."""
    with open(path, 'rb') as f:
        raw = f.read()
    pos = 0

    def token():
        nonlocal pos
        while pos < len(raw) and raw[pos:pos + 1].isspace():
            pos += 1
        if pos < len(raw) and raw[pos:pos + 1] == b'#':
            while pos < len(raw) and raw[pos:pos + 1] != b'\n':
                pos += 1
            return token()
        start = pos
        while pos < len(raw) and not raw[pos:pos + 1].isspace():
            pos += 1
        return raw[start:pos]

    if token() != b'P5':
        raise ValueError('not a binary (P5) pgm: {}'.format(path))
    width = int(token())
    height = int(token())
    int(token())  # maxval
    pos += 1
    data = np.frombuffer(raw[pos:pos + width * height], dtype=np.uint8)
    return data.reshape(height, width)


def load_tum(path):
    rows = []
    for line in open(path):
        p = line.split()
        if len(p) >= 3:
            try:
                rows.append((float(p[0]), float(p[1]), float(p[2])))
            except ValueError:
                continue
    return rows


def errors(gt, est, max_diff=0.1):
    """Per-estimate translation error to the nearest-in-time ground-truth sample."""
    out = []
    j = 0
    for te, xe, ye in est:
        while j + 1 < len(gt) and abs(gt[j + 1][0] - te) <= abs(gt[j][0] - te):
            j += 1
        tg, xg, yg = gt[j]
        out.append(((xe - xg) ** 2 + (ye - yg) ** 2) ** 0.5 if abs(tg - te) <= max_diff else 0.0)
    return out


def main():
    if len(sys.argv) < 5:
        sys.stderr.write('usage: plot_trajectory_on_map.py gt est map.yaml out.png [title]\n')
        return 2
    gt_f, est_f, yaml_f, out = sys.argv[1:5]
    title = sys.argv[5] if len(sys.argv) > 5 else 'emcl2 estimate vs ground truth on map'

    meta = read_yaml(yaml_f)
    res = float(meta['resolution'])
    origin = meta['origin'].strip('[]').split(',')
    ox, oy = float(origin[0]), float(origin[1])
    img_path = meta['image'].strip('"\'')
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_f)), img_path)
    grid = read_pgm(img_path)
    h, w = grid.shape

    gt = load_tum(gt_f)
    est = load_tum(est_f)
    if not gt or not est:
        sys.stderr.write('[plot] empty trajectory\n')
        return 1
    errs = errors(gt, est)

    fig, ax = plt.subplots(figsize=(6.5, 6.0))
    ax.imshow(
        grid, cmap='gray', vmin=0, vmax=255, origin='upper',
        extent=[ox, ox + w * res, oy, oy + h * res])
    ax.plot([p[1] for p in gt], [p[2] for p in gt], 'g--', linewidth=1.0, label='ground truth')
    sc = ax.scatter(
        [p[1] for p in est], [p[2] for p in est], c=errs, cmap='jet', s=8, vmin=0.0,
        label='estimate (APE)')
    fig.colorbar(sc, ax=ax, label='translation error [m]')

    # Zoom to the travelled area (plus a margin) rather than the whole map.
    xs = [p[1] for p in gt] + [p[1] for p in est]
    ys = [p[2] for p in gt] + [p[2] for p in est]
    m = 1.0
    ax.set_xlim(min(xs) - m, max(xs) + m)
    ax.set_ylim(min(ys) - m, max(ys) + m)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(title)
    ax.legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=100)
    sys.stderr.write('[plot] wrote {}\n'.format(out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
