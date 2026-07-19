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
Generate the occupancy map for worlds/symmetric_room.sdf.xacro.

Rasterises the SAME wall / pillar rectangles as the world SDF so the map and the
simulated geometry cannot drift apart. Writes symmetric_room.pgm (P5) next to
this script. Run it whenever the room geometry changes:

    python3 maps/generate_symmetric_room.py

Occupancy encoding (nav2 map_server, negate=0, default thresholds): occupied
cells are 0 (black), free cells 254 (white), and everything outside the outer
walls is a mid-grey 128 that map_server reads as unknown, so global
localization seeds particles only inside the room.
"""

import os

RESOLUTION = 0.05
X_MIN, X_MAX = -4.5, 4.5
Y_MIN, Y_MAX = -2.5, 2.5

# (centre_x, centre_y, size_x, size_y) -- identical to the world SDF boxes.
OBSTACLES = [
    (-4.0, 0.0, 0.1, 4.1),   # wall_left
    (4.0, 0.0, 0.1, 4.1),    # wall_right
    (0.0, -2.0, 8.1, 0.1),   # wall_bottom
    (0.0, 2.0, 8.1, 0.1),    # wall_top
    (2.0, 0.0, 0.4, 0.4),    # pillar_a
    (-2.0, 0.0, 0.4, 0.4),   # pillar_b
]
# Interior half-extents (inside the inner wall faces).
INNER_X = 3.95
INNER_Y = 1.95

OCCUPIED, FREE, UNKNOWN = 0, 254, 128


def in_rect(x, y, cx, cy, sx, sy):
    return abs(x - cx) <= sx / 2 and abs(y - cy) <= sy / 2


def classify(x, y):
    if any(in_rect(x, y, *o) for o in OBSTACLES):
        return OCCUPIED
    if abs(x) <= INNER_X and abs(y) <= INNER_Y:
        return FREE
    return UNKNOWN


def main():
    width = int(round((X_MAX - X_MIN) / RESOLUTION))
    height = int(round((Y_MAX - Y_MIN) / RESOLUTION))
    pixels = bytearray()
    for row in range(height):
        # PGM rows run top (max y) to bottom (min y).
        y = Y_MAX - (row + 0.5) * RESOLUTION
        for col in range(width):
            x = X_MIN + (col + 0.5) * RESOLUTION
            pixels.append(classify(x, y))

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'symmetric_room.pgm')
    with open(out, 'wb') as f:
        f.write('P5\n{} {}\n255\n'.format(width, height).encode())
        f.write(bytes(pixels))
    print('wrote {} ({}x{} cells, {} m/cell)'.format(out, width, height, RESOLUTION))


if __name__ == '__main__':
    main()
