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
Report the motion the robot actually executed, from the ground-truth TUM file.

Differentiates the ground-truth pose over a short window to get the peak linear
and angular speed, plus the path length and duration. Printed for the evo PR
comment so the driven trajectory's aggressiveness is on record.

Usage: motion_stats.py <ground_truth.tum>
"""

import math
import sys


def yaw(qz, qw):
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


def load(path):
    rows = []
    for line in open(path):
        p = line.split()
        if len(p) >= 8:
            try:
                rows.append((float(p[0]), float(p[1]), float(p[2]), yaw(float(p[6]), float(p[7]))))
            except ValueError:
                continue
    return rows


def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    if len(sys.argv) < 2:
        sys.stderr.write('usage: motion_stats.py ground_truth.tum\n')
        return 2
    rows = load(sys.argv[1])
    if len(rows) < 4:
        sys.stderr.write('[motion] too few samples\n')
        return 1

    # Differentiate over a ~0.15 s window to suppress single-sample noise.
    dt_total = rows[-1][0] - rows[0][0]
    hz = (len(rows) - 1) / dt_total if dt_total > 0 else 30.0
    k = max(1, int(round(0.15 * hz)))

    max_v = 0.0
    max_w = 0.0
    for i in range(k, len(rows)):
        t0, x0, y0, a0 = rows[i - k]
        t1, x1, y1, a1 = rows[i]
        dt = t1 - t0
        if dt <= 0:
            continue
        max_v = max(max_v, math.hypot(x1 - x0, y1 - y0) / dt)
        max_w = max(max_w, abs(norm_angle(a1 - a0)) / dt)

    length = sum(
        math.hypot(rows[i][1] - rows[i - 1][1], rows[i][2] - rows[i - 1][2])
        for i in range(1, len(rows)))

    print('max linear velocity  : {:.3f} m/s'.format(max_v))
    print('max angular velocity : {:.3f} rad/s'.format(max_w))
    print('path length          : {:.2f} m'.format(length))
    print('duration             : {:.1f} s'.format(dt_total))
    return 0


if __name__ == '__main__':
    sys.exit(main())
