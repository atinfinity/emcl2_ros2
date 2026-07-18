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

#ifndef EMCL2__MCL_HPP_
#define EMCL2__MCL_HPP_

#include <memory>
#include <random>
#include <sstream>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "emcl2/LikelihoodFieldMap.hpp"
#include "emcl2/OdomModel.hpp"
#include "emcl2/Particle.hpp"

namespace emcl2
{
class Mcl
{
public:
  Mcl(
    const Pose & p, int num, const Scan & scan,
    const std::shared_ptr<OdomModel> & odom_model,
    const std::shared_ptr<LikelihoodFieldMap> & map);
  virtual ~Mcl();

  std::vector<Particle> particles_;
  double alpha_;

  virtual void sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv) = 0;
  void motionUpdate(double x, double y, double t);

  void initialize(double x, double y, double t);

  void setScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void setMap(const std::shared_ptr<LikelihoodFieldMap> & map);
  void meanPose(
    double & x_mean, double & y_mean, double & t_mean, double & x_var, double & y_var,
    double & t_var, double & xy_cov, double & yt_cov, double & tx_cov);

  void simpleReset(void);
  void clearProcessedScan(void) {processed_seq_ = -1;}

  static double cos_[(1 << 16)];
  static double sin_[(1 << 16)];

protected:
  std::unique_ptr<Pose> last_odom_;
  std::unique_ptr<Pose> prev_odom_;

  Scan scan_;
  int processed_seq_;

  std::mt19937 rng_ {std::random_device{}()};

  double normalizeAngle(double t);
  void resampling(void);
  double normalizeBelief(void);
  void resetWeight(void);

  std::shared_ptr<OdomModel> odom_model_;
  std::shared_ptr<LikelihoodFieldMap> map_;
};

}  // namespace emcl2

#endif  // EMCL2__MCL_HPP_
