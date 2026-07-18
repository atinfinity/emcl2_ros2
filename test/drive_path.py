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
SCHEDULE = [
    (0.15, 0.0, 6.0),    # forward
    (0.0, 0.5, 3.0),     # turn left in place
    (0.15, 0.0, 6.0),    # forward
    (0.0, -0.5, 3.0),    # turn right in place
    (0.15, 0.3, 8.0),    # forward + gentle arc
    (0.0, 0.6, 3.0),     # turn
    (0.15, 0.0, 6.0),    # forward
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
