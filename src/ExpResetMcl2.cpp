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

#include "emcl2/ExpResetMcl2.hpp"

#include <cmath>
#include <iostream>
#include <random>

#include <rclcpp/rclcpp.hpp>

namespace emcl2
{
ExpResetMcl2::ExpResetMcl2(
  const Pose & p, int num, const Scan & scan, const std::shared_ptr<OdomModel> & odom_model,
  const std::shared_ptr<LikelihoodFieldMap> & map, double alpha_th,
  double expansion_radius_position, double expansion_radius_orientation, double extraction_rate,
  double range_threshold, bool sensor_reset, bool enable_expansion_resetting)
: Mcl::Mcl(p, num, scan, odom_model, map),
  alpha_threshold_(alpha_th),
  expansion_radius_position_(expansion_radius_position),
  expansion_radius_orientation_(expansion_radius_orientation),
  extraction_rate_(extraction_rate),
  range_threshold_(range_threshold),
  sensor_reset_(sensor_reset),
  enable_expansion_resetting_(enable_expansion_resetting)
{
}

ExpResetMcl2::~ExpResetMcl2() {}

void ExpResetMcl2::sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv)
{
  if (processed_seq_ == scan_.seq_) {
    return;
  }

  Scan scan;
  scan = scan_;

  scan.lidar_pose_x_ = lidar_x;
  scan.lidar_pose_y_ = lidar_y;
  scan.lidar_pose_yaw_ = lidar_t;

  double origin = inv ? scan.angle_max_ : scan.angle_min_;
  int sgn = inv ? -1 : 1;
  for(size_t i = 0; i < scan.ranges_.size() ; i++) {
    scan.directions_16bit_.push_back(Pose::get16bitRepresentation(
                        origin + sgn * i * scan.angle_increment_));
  }

  double valid_pct = 0.0;
  int valid_beams = scan.countValidBeams(&valid_pct);
  if (valid_beams == 0) {
    return;
  }

  const int num = static_cast<int>(particles_.size());
#ifdef _OPENMP
  #pragma omp parallel for
#endif
  for (int i = 0; i < num; i++) {
    particles_[i].w_ *= particles_[i].likelihood(map_.get(), scan);
  }

  alpha_ = nonPenetrationRate(static_cast<int>(particles_.size() * extraction_rate_), map_.get(),
      scan);
  RCLCPP_DEBUG(rclcpp::get_logger("emcl2_node"), "ALPHA: %f / %f", alpha_, alpha_threshold_);
  if (enable_expansion_resetting_ && alpha_ < alpha_threshold_) {
    RCLCPP_INFO(rclcpp::get_logger("emcl2_node"), "RESET");
    expansionReset();
#ifdef _OPENMP
    #pragma omp parallel for
#endif
    for (int i = 0; i < num; i++) {
      particles_[i].w_ *= particles_[i].likelihood(map_.get(), scan);
    }
  }

  if (normalizeBelief() > 0.000001) {
    resampling();
  } else {
    resetWeight();
  }

  processed_seq_ = scan_.seq_;
}

double ExpResetMcl2::nonPenetrationRate(int skip, LikelihoodFieldMap * map, Scan & scan)
{
  if (skip < 1) {
    skip = 1;
  }

  static uint16_t shift = 0;
  const int start = shift % skip;
  const int n = (static_cast<int>(particles_.size()) - start + skip - 1) / skip;
  int counter = 0;
  int penetrating = 0;
#ifdef _OPENMP
  #pragma omp parallel for schedule(dynamic) reduction(+ : counter, penetrating)
#endif
  for (int k = 0; k < n; k++) {
    counter++;
    if (particles_[start + k * skip].wallConflict(
        map, scan, range_threshold_, sensor_reset_, phantom_robust_))
    {
      penetrating++;
    }
  }
  shift++;

  RCLCPP_DEBUG(rclcpp::get_logger("emcl2_node"), "%d %d", penetrating, counter);
  return static_cast<double>((counter - penetrating)) / counter;
}

void ExpResetMcl2::expansionReset(void)
{
  std::uniform_real_distribution<double> ud(-1.0, 1.0);
  for (auto & p : particles_) {
    double length = ud(rng_) * expansion_radius_position_;
    double direction = ud(rng_) * M_PI;

    p.p_.x_ += length * cos(direction);
    p.p_.y_ += length * sin(direction);
    p.p_.t_ += ud(rng_) * expansion_radius_orientation_;
    p.w_ = 1.0 / particles_.size();
  }
}

}  // namespace emcl2
