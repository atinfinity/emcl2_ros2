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

#ifndef EMCL2__EXPRESETMCL2_HPP_
#define EMCL2__EXPRESETMCL2_HPP_

#include <memory>

#include "emcl2/Mcl.hpp"

namespace emcl2
{
class ExpResetMcl2 : public Mcl
{
public:
  ExpResetMcl2(
    const Pose & p, int num, const Scan & scan,
    const std::shared_ptr<OdomModel> & odom_model,
    const std::shared_ptr<LikelihoodFieldMap> & map, double alpha_th,
    double expansion_radius_position, double expansion_radius_orientation,
    double extraction_rate, double successive_penetration_threshold, bool sensor_reset,
    bool enable_expansion_resetting);
  ~ExpResetMcl2();

  void sensorUpdate(double lidar_x, double lidar_y, double lidar_t, bool inv) override;

private:
  double alpha_threshold_;
  double expansion_radius_position_;
  double expansion_radius_orientation_;

  double extraction_rate_;
  double range_threshold_;
  bool sensor_reset_;
  bool enable_expansion_resetting_;

  void expansionReset(void);

  // bool Particle::isPenetrating(
  double nonPenetrationRate(int skip, LikelihoodFieldMap * map, Scan & scan);
};

}  // namespace emcl2

#endif  // EMCL2__EXPRESETMCL2_HPP_
