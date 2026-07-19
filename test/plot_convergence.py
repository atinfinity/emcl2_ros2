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
Plot the translation-error-over-time of every recovery run in a directory.

Reads run*/ground_truth.tum and run*/mcl_pose.tum under the given parent dir,
associates estimate to ground truth by nearest timestamp, and overlays each
run's error(t) (t relative to that run's first logged sample) on one axes with
the convergence threshold line. This visualises how consistently -- and how
fast -- emcl2 recovers across the N runs. Saves a single PNG (Agg backend).

Usage: plot_convergence.py <parent_dir> <out.png> [threshold_m] [title]
"""

import glob
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402


def load(path):
    rows = []
    try:
        with open(path) as fh:
            for line in fh:
                p = line.split()
                if len(p) >= 3:
                    try:
                        rows.append((float(p[0]), float(p[1]), float(p[2])))
                    except ValueError:
                        continue
    except OSError:
        return []
    return rows


def error_series(gt, est, max_diff=0.1):
    """Nearest-timestamp error(t) with t relative to the first estimate sample."""
    series = []
    j = 0
    for te, xe, ye in est:
        while j + 1 < len(gt) and abs(gt[j + 1][0] - te) <= abs(gt[j][0] - te):
            j += 1
        tg, xg, yg = gt[j]
        if abs(tg - te) <= max_diff:
            series.append((te, ((xe - xg) ** 2 + (ye - yg) ** 2) ** 0.5))
    if not series:
        return [], []
    t0 = series[0][0]
    return [t - t0 for t, _ in series], [e for _, e in series]


def main():
    if len(sys.argv) < 3:
        sys.stderr.write('usage: plot_convergence.py parent_dir out.png [threshold] [title]\n')
        return 2
    parent, out = sys.argv[1], sys.argv[2]
    threshold = float(sys.argv[3]) if len(sys.argv) > 3 else 0.3
    title = sys.argv[4] if len(sys.argv) > 4 else 'recovery convergence'

    fig, ax = plt.subplots(figsize=(7.0, 4.0))
    plotted = 0
    for run_dir in sorted(glob.glob(os.path.join(parent, 'run*'))):
        ts, errs = error_series(
            load(os.path.join(run_dir, 'ground_truth.tum')),
            load(os.path.join(run_dir, 'mcl_pose.tum')))
        if not ts:
            continue
        ax.plot(ts, errs, alpha=0.75, linewidth=1.2, label=os.path.basename(run_dir))
        plotted += 1

    ax.axhline(threshold, color='k', linestyle='--', linewidth=1.0,
               label=f'threshold {threshold:g} m')
    ax.set_xlabel('time since logging start [s]')
    ax.set_ylabel('translation error [m]')
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(bottom=0.0)
    if plotted:
        ax.legend(fontsize=8, loc='upper right')
    fig.tight_layout()
    fig.savefig(out, dpi=100)
    sys.stderr.write(f'[plot] wrote {out} ({plotted} runs)\n')
    return 0 if plotted else 1


if __name__ == '__main__':
    sys.exit(main())
