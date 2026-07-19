// Copyright 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: LGPL-3.0-or-later
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "emcl2/Particle.hpp"

#include <cmath>
#include <random>
#include <vector>

#include "emcl2/Mcl.hpp"

namespace emcl2
{
Particle::Particle(double x, double y, double t, double w)
: p_(x, y, t) {w_ = w;}

double Particle::likelihood(LikelihoodFieldMap * map, Scan & scan)
{
  uint16_t t = p_.get16bitRepresentation();
  double lidar_x =
    p_.x_ + scan.lidar_pose_x_ * Mcl::cos_[t] - scan.lidar_pose_y_ * Mcl::sin_[t];
  double lidar_y =
    p_.y_ + scan.lidar_pose_x_ * Mcl::sin_[t] + scan.lidar_pose_y_ * Mcl::cos_[t];
  uint16_t lidar_yaw = Pose::get16bitRepresentation(scan.lidar_pose_yaw_);

  double ans = 0.0;
  for (size_t i = 0; i < scan.ranges_.size(); i += scan.scan_increment_) {
    if (!scan.valid(scan.ranges_[i])) {
      continue;
    }
    uint16_t a = scan.directions_16bit_[i] + t + lidar_yaw;
    double lx = lidar_x + scan.ranges_[i] * Mcl::cos_[a];
    double ly = lidar_y + scan.ranges_[i] * Mcl::sin_[a];

    ans += map->likelihood(lx, ly);
  }
  return ans;
}

bool Particle::wallConflict(
  LikelihoodFieldMap * map, Scan & scan, double threshold, bool replace, bool endpoint_check)
{
  uint16_t t = p_.get16bitRepresentation();
  double lidar_x =
    p_.x_ + scan.lidar_pose_x_ * Mcl::cos_[t] - scan.lidar_pose_y_ * Mcl::sin_[t];
  double lidar_y =
    p_.y_ + scan.lidar_pose_x_ * Mcl::sin_[t] + scan.lidar_pose_y_ * Mcl::cos_[t];
  uint16_t lidar_yaw = Pose::get16bitRepresentation(scan.lidar_pose_yaw_);

  static thread_local std::mt19937 rng {std::random_device{}()};
  std::vector<int> order;
  if (rng() % 2) {
    for (size_t i = 0; i < scan.ranges_.size(); i += scan.scan_increment_) {
      order.push_back(i);
    }
  } else {
    for (int i = scan.ranges_.size() - 1; i >= 0; i -= scan.scan_increment_) {
      order.push_back(i);
    }
  }

  int hit_counter = 0;
  double hit_lx1 = 0.0, hit_ly1 = 0.0, r1 = 0.0;
  uint16_t a1 = 0;
  for (int i : order) {
    if (!scan.valid(scan.ranges_[i])) {
      continue;
    }

    double range = scan.ranges_[i];
    uint16_t a = scan.directions_16bit_[i] + t + lidar_yaw;

    double hit_lx = 0.0, hit_ly = 0.0;
    if (isPenetrating(lidar_x, lidar_y, range, a, map, hit_lx, hit_ly, endpoint_check)) {
      if (hit_counter == 0) {
        hit_lx1 = hit_lx;
        hit_ly1 = hit_ly;
        r1 = range;
        a1 = a;
      }

      hit_counter++;
    } else {
      hit_counter = 0;
    }

    if (hit_counter * scan.angle_increment_ >= threshold) {
      if (replace) {
        sensorReset(
                                  lidar_x, lidar_y, r1, a1, hit_lx1, hit_ly1, range, a, hit_lx,
                                  hit_ly);
      }
      return true;
    }
  }
  return false;
}

bool Particle::isPenetrating(
  double ox, double oy, double range, uint16_t direction, LikelihoodFieldMap * map, double & hit_lx,
  double & hit_ly, bool endpoint_check)
{
  bool hit = false;
  for (double d = map->resolution_; d < range; d += map->resolution_) {
    double lx = ox + d * Mcl::cos_[direction];
    double ly = oy + d * Mcl::sin_[direction];

    if ((!hit) && map->likelihood(lx, ly) == 255) {
      hit = true;
      hit_lx = lx;
      hit_ly = ly;
    } else if (hit && map->likelihood(lx, ly) == 0.0) {              // openspace after hit
      // The beam crossed a mapped obstacle and then open space. If it still ends
      // (at the measured range) on a mapped obstacle, the scan is explained by a
      // real wall behind the crossed one -- typically a stale/phantom obstacle
      // in the map -- so this is not a genuine penetration. Only flag it when the
      // endpoint is unexplained (open space). Guards against spurious expansion
      // resets from an out-of-date map while still catching real conflicts.
      if (endpoint_check) {
        double ex = ox + range * Mcl::cos_[direction];
        double ey = oy + range * Mcl::sin_[direction];
        if (map->likelihood(ex, ey) == 255) {  // endpoint exactly on a mapped wall
          return false;
        }
      }
      return true;                                                   // penetration
    }
  }
  return false;
}

void Particle::sensorReset(
  double ox, double oy, double range1, uint16_t direction1, double hit_lx1, double hit_ly1,
  double range2, uint16_t direction2, double hit_lx2, double hit_ly2)
{
  double p1_x = ox + range1 * Mcl::cos_[direction1];
  double p1_y = oy + range1 * Mcl::sin_[direction1];
  double p2_x = ox + range2 * Mcl::cos_[direction2];
  double p2_y = oy + range2 * Mcl::sin_[direction2];

  double cx = (hit_lx1 + hit_lx2) / 2;
  double cy = (hit_ly1 + hit_ly2) / 2;

  p_.x_ -= (p1_x + p2_x) / 2 - cx;
  p_.y_ -= (p1_y + p2_y) / 2 - cy;

  double theta_delta =
    atan2(p2_y - p1_y, p2_x - p1_x) - atan2(hit_ly2 - hit_ly1, hit_lx2 - hit_lx1);

        // double d = std::sqrt((p_.x_ - cx)*(p_.x_ - cx) + (p_.y_ - cy)*(p_.y_ - cy));

        // double theta = atan2(p_.y_ - cy, p_.x_ - cx) - theta_delta;
        // p_.x_ = cx + d * std::cos(theta);
        // p_.y_ = cy + d * std::cos(theta);

  p_.t_ -= theta_delta;
}

}  // namespace emcl2
