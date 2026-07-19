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
Drive the robot around the whole map along a fixed set of waypoints.

Used by the localization CI job. A closed-loop controller steers the robot
through WAYPOINTS using the Gazebo ground-truth pose (so the path is
deterministic and reproducible), following a lawnmower tour of the tb3_sandbox
free space that avoids the nine pillars -- the earlier fixed cmd_vel schedule
only wandered a local patch and clipped obstacles. It publishes
geometry_msgs/Twist on /cmd_vel and exits when the last waypoint is reached (or
after a safety timeout), leaving the robot stopped.
"""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

# Lawnmower tour of the tb3_sandbox free lanes (world metres). Each straight
# segment was checked clear of the pillars and walls for a 0.22 m robot radius;
# it spans x in [-1.65, 1.65], y in [-1.6, 1.6], i.e. the whole drivable area.
WAYPOINTS = [
    (-1.65, -1.6), (-1.65, 1.6),
    (-0.55, 1.6), (-0.55, -1.6),
    (0.55, -1.6), (0.55, 1.6),
    (1.65, 1.6), (1.65, -1.6),
]

MAX_LIN = 0.22          # m/s
MAX_ANG = 1.0           # rad/s
K_LIN = 0.8
K_ANG = 1.6
REACH_TOL = 0.15        # m: waypoint considered reached
ALIGN_TOL = 0.30        # rad: rotate in place until roughly facing the target
RATE_HZ = 20.0
TIMEOUT_S = 300.0       # wall-clock safety stop


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class DrivePath(Node):
    def __init__(self):
        super().__init__('drive_path')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'ground_truth', self.on_odom, 10)
        self.pose = None
        self.idx = 0
        self.ticks = 0
        self.timer = self.create_timer(1.0 / RATE_HZ, self.on_timer)
        self.get_logger().info('drive_path started')

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

        x, y, yaw = self.pose
        tx, ty = WAYPOINTS[self.idx]
        dx, dy = tx - x, ty - y
        dist = math.hypot(dx, dy)
        if dist < REACH_TOL:
            self.idx += 1
            if self.idx >= len(WAYPOINTS):
                self.get_logger().info('drive_path finished')
                self._finish()
            return

        heading_err = math.atan2(math.sin(math.atan2(dy, dx) - yaw),
                                 math.cos(math.atan2(dy, dx) - yaw))
        cmd = Twist()
        if abs(heading_err) > ALIGN_TOL:
            cmd.linear.x = 0.0  # rotate in place first
        else:
            cmd.linear.x = clamp(K_LIN * dist, 0.0, MAX_LIN)
        cmd.angular.z = clamp(K_ANG * heading_err, -MAX_ANG, MAX_ANG)
        self.pub.publish(cmd)

    def _finish(self):
        self.pub.publish(Twist())
        raise SystemExit


def main():
    rclpy.init()
    node = DrivePath()
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
