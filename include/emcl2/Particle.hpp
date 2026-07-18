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

#ifndef EMCL2__PARTICLE_HPP_
#define EMCL2__PARTICLE_HPP_

#include "emcl2/LikelihoodFieldMap.hpp"
#include "emcl2/Pose.hpp"

namespace emcl2
{
class Particle
{
public:
  Particle(double x, double y, double t, double w);
  Particle(const Particle & other) = default;

  Particle & operator=(const Particle & other)
  {
    if (this != &other) {
      this->p_ = other.p_;
      this->w_ = other.w_;
    }
    return *this;
  }

  double likelihood(LikelihoodFieldMap * map, Scan & scan);
  bool wallConflict(LikelihoodFieldMap * map, Scan & scan, double threshold, bool replace);
  Pose p_;
  double w_;

private:
  bool isPenetrating(
    double ox, double oy, double range, uint16_t direction, LikelihoodFieldMap * map,
    double & hit_lx, double & hit_ly);

  bool checkWallConflict(
    LikelihoodFieldMap * map, double ox, double oy, double range, uint16_t direction,
    double threshold, bool replace);

  void sensorReset(
    double ox, double oy, double range1, uint16_t direction1, double hit_lx1, double hit_ly1,
    double range2, uint16_t direction2, double hit_lx2, double hit_ly2);
};

}  // namespace emcl2

#endif  // EMCL2__PARTICLE_HPP_
