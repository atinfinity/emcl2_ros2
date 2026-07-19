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
Drive the robot along a fixed, deterministic cmd_vel trajectory.

Used by the localization CI job so that every run follows the same path
(straight + turning segments), which keeps the evo evaluation reproducible.
It publishes geometry_msgs/Twist on /cmd_vel and exits when the schedule is
done, leaving the robot stopped.
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

# (linear.x [m/s], angular.z [rad/s], duration [s]) segments.
# A demanding but collision-conscious path: faster translation, brisk in-place
# spins in both directions and tight arcs stress the motion model and the
# rotation handling, and the extra length gives the evaluation more samples so
# small accuracy changes are easier to separate from run-to-run noise.
SCHEDULE = [
    (0.20, 0.0, 3.5),    # forward
    (0.16, 0.9, 5.5),    # arc left (translate + turn)
    (0.0, 1.0, 2.0),     # short spin to reorient
    (0.20, -0.6, 5.5),   # arc right
    (0.0, -1.1, 2.0),    # short fast spin
    (0.18, 0.8, 5.5),    # tight arc left
    (0.20, 0.0, 3.5),    # forward
    (0.0, 1.0, 2.0),     # spin
    (0.16, -0.9, 5.5),   # tight arc right
    (0.20, 0.0, 3.0),    # forward
    (0.0, -1.0, 2.0),    # spin
    (0.18, 0.7, 5.0),    # arc
]


class DrivePath(Node):
    def __init__(self):
        super().__init__('drive_path')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.seg = 0
        self.ticks = 0
        self.rate_hz = 20.0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)
        self.get_logger().info('drive_path started')

    def on_timer(self):
        if self.seg >= len(SCHEDULE):
            self.pub.publish(Twist())  # stop
            self.get_logger().info('drive_path finished')
            raise SystemExit
        lin, ang, dur = SCHEDULE[self.seg]
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub.publish(msg)
        self.ticks += 1
        if self.ticks >= int(dur * self.rate_hz):
            self.seg += 1
            self.ticks = 0


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
