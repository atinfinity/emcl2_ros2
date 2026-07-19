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
Drive the robot along a fixed, collision-free trajectory for the eval.

Used by the localization CI job. A closed-loop controller steers the robot
using the Gazebo ground-truth pose (so the path is deterministic and
reproducible). The trajectory shape is chosen by the first CLI argument so the
evo evaluation can exercise several motion profiles, not just straight lines:

  full  (default) lawnmower tour of the whole tb3_sandbox free space -- mostly
                  straight legs with in-place turns at the ends.
  arcs            orbit a clear circle several times: a sustained arc where the
                  robot translates and rotates at once (v>0 and w>0).
  spin            in-place rotation only at the spawn: pure yaw, no translation.

Every trajectory keeps clear of the nine pillars and the walls for a 0.22 m
robot radius (the arc circle and the spin footprint were checked against the
map). It publishes geometry_msgs/Twist on /cmd_vel and exits when the
trajectory is complete (or after a safety timeout), leaving the robot stopped.
"""

import math
import sys

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

# --- full: lawnmower tour of the tb3_sandbox free lanes (world metres). Each
# straight segment was checked clear of the pillars and walls for a 0.22 m robot
# radius; it spans x in [-1.65, 1.65], y in [-1.6, 1.6], the whole drivable area.
FULL_WAYPOINTS = [
    (-1.65, -1.6), (-1.65, 1.6),
    (-0.55, 1.6), (-0.55, -1.6),
    (0.55, -1.6), (0.55, 1.6),
    (1.65, 1.6), (1.65, -1.6),
]

# --- arcs: orbit a circle that fits entirely in the free strip between the left
# wall and the pillar columns. The largest clear circle centred there has radius
# ~0.45 m for a 0.22 m robot, so 0.40 m leaves a margin on every point.
ARC_CENTER = (-2.0, 0.0)
ARC_RADIUS = 0.40
ARC_LAPS = 2.0
ARC_LIN = 0.18          # m/s along the circle -> w ~= 0.45 rad/s (v = w * r)
ARC_LOOKAHEAD = 0.35    # rad: carrot angle ahead of the robot on the circle

# --- spin: pure in-place rotation at the spawn, both directions.
SPIN_ROTATIONS = 2.0    # full turns per direction

MAX_LIN = 0.22          # m/s
MAX_ANG = 1.0           # rad/s
K_LIN = 0.8
K_ANG = 1.6
REACH_TOL = 0.15        # m: waypoint considered reached
ALIGN_TOL = 0.30        # rad: rotate in place until roughly facing the target
RATE_HZ = 20.0
TIMEOUT_S = 300.0       # wall-clock safety stop
TWO_PI = 2.0 * math.pi


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class DrivePath(Node):
    def __init__(self, mode):
        super().__init__('drive_path')
        self.mode = mode
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'ground_truth', self.on_odom, 10)
        self.pose = None
        self.idx = 0
        self.ticks = 0
        # arc/spin progress state
        self.orbiting = False
        self.swept = 0.0
        self.prev_ang = None
        self.spin_dir = 1.0
        self.timer = self.create_timer(1.0 / RATE_HZ, self.on_timer)
        self.get_logger().info(f'drive_path started (mode={mode})')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def on_timer(self):
        self.ticks += 1
        if self.ticks > TIMEOUT_S * RATE_HZ:
            self.get_logger().warn('drive_path timeout; stopping')
            self._finish()
        if self.pose is None:
            return  # wait for the first ground-truth pose
        if self.mode == 'arcs':
            self.step_arcs()
        elif self.mode == 'spin':
            self.step_spin()
        else:
            self.step_full()

    def steer_to(self, tx, ty, forward=True):
        """Publish a Twist that drives toward (tx, ty). Returns the distance."""
        x, y, yaw = self.pose
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        heading_err = wrap(math.atan2(dy, dx) - yaw)
        cmd = Twist()
        if not forward or abs(heading_err) > ALIGN_TOL:
            cmd.linear.x = 0.0  # rotate in place until roughly facing the target
        else:
            cmd.linear.x = clamp(K_LIN * dist, 0.0, MAX_LIN)
        cmd.angular.z = clamp(K_ANG * heading_err, -MAX_ANG, MAX_ANG)
        self.pub.publish(cmd)
        return dist

    def step_full(self):
        tx, ty = FULL_WAYPOINTS[self.idx]
        if self.steer_to(tx, ty) < REACH_TOL:
            self.idx += 1
            if self.idx >= len(FULL_WAYPOINTS):
                self.get_logger().info('drive_path finished')
                self._finish()

    def step_arcs(self):
        x, y, yaw = self.pose
        cx, cy = ARC_CENTER
        if not self.orbiting:
            # Approach the entry point on the circle nearest the spawn, then orbit.
            ang0 = math.atan2(y - cy, x - cx)
            ex, ey = cx + ARC_RADIUS * math.cos(ang0), cy + ARC_RADIUS * math.sin(ang0)
            if self.steer_to(ex, ey) < REACH_TOL:
                self.orbiting = True
                self.prev_ang = math.atan2(y - cy, x - cx)
            return
        ang = math.atan2(y - cy, x - cx)
        self.swept += wrap(ang - self.prev_ang)   # CCW carrot -> accumulates positive
        self.prev_ang = ang
        if self.swept >= ARC_LAPS * TWO_PI:
            self.get_logger().info('drive_path finished')
            self._finish()
            return
        # Carrot a fixed angle ahead on the circle; steering to it holds the radius.
        carrot = ang + ARC_LOOKAHEAD
        tx, ty = cx + ARC_RADIUS * math.cos(carrot), cy + ARC_RADIUS * math.sin(carrot)
        heading_err = wrap(math.atan2(ty - y, tx - x) - yaw)
        cmd = Twist()
        cmd.linear.x = ARC_LIN
        cmd.angular.z = clamp(K_ANG * heading_err, -MAX_ANG, MAX_ANG)
        self.pub.publish(cmd)

    def step_spin(self):
        _, _, yaw = self.pose
        if self.prev_ang is None:
            self.prev_ang = yaw
            return
        self.swept += abs(wrap(yaw - self.prev_ang))
        self.prev_ang = yaw
        if self.swept >= SPIN_ROTATIONS * TWO_PI:
            if self.spin_dir > 0.0:
                self.spin_dir = -1.0     # reverse once, then repeat the sweep
                self.swept = 0.0
            else:
                self.get_logger().info('drive_path finished')
                self._finish()
                return
        cmd = Twist()
        cmd.angular.z = self.spin_dir * MAX_ANG   # pure rotation, no translation
        self.pub.publish(cmd)

    def _finish(self):
        self.pub.publish(Twist())
        raise SystemExit


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'full'
    if mode not in ('full', 'arcs', 'spin'):
        print(f'unknown drive mode {mode!r}; use full|arcs|spin', file=sys.stderr)
        raise SystemExit(2)
    rclpy.init()
    node = DrivePath(mode)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
