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
Log ground-truth and estimated poses to TUM trajectory files for evo.

Subscribes to /ground_truth (nav_msgs/Odometry) and /mcl_pose
(geometry_msgs/PoseWithCovarianceStamped) and appends a TUM line
("timestamp tx ty tz qx qy qz qw") for each message. Every line is flushed,
so the files stay valid even if the node is killed — this avoids the rosbag2
finalize step that is fragile under heavy load in CI.

Usage: pose_logger.py <gt_out.tum> <est_out.tum>
"""

import sys

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def tum_line(stamp, p, q):
    t = stamp.sec + stamp.nanosec * 1e-9
    return f'{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n'


class PoseLogger(Node):
    def __init__(self, gt_path, est_path):
        super().__init__('pose_logger')
        self.gt_f = open(gt_path, 'w')
        self.est_f = open(est_path, 'w')
        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(Odometry, 'ground_truth', self.on_gt, qos)
        self.create_subscription(
            PoseWithCovarianceStamped, 'mcl_pose', self.on_est, qos)
        self.n_gt = 0
        self.n_est = 0

    def on_gt(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.gt_f.write(tum_line(msg.header.stamp, p, q))
        self.gt_f.flush()
        self.n_gt += 1

    def on_est(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.est_f.write(tum_line(msg.header.stamp, p, q))
        self.est_f.flush()
        self.n_est += 1

    def close(self):
        self.get_logger().info(f'logged gt={self.n_gt} est={self.n_est}')
        self.gt_f.close()
        self.est_f.close()


def main():
    gt_path = sys.argv[1]
    est_path = sys.argv[2]
    rclpy.init()
    node = PoseLogger(gt_path, est_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
