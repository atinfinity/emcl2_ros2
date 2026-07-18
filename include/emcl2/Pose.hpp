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

#ifndef EMCL2__POSE_HPP_
#define EMCL2__POSE_HPP_

#include <sstream>
#include <string>
#include <cstdint>

namespace emcl2
{

class Pose
{
public:
  Pose()
  {
  }
  Pose(double x, double y, double t);
  Pose(const Pose & other);

  void set(double x, double y, double t);
  void set(const Pose & p);
  std::string to_s(void);

  void normalizeAngle(void);
  void move(
    double length, double direction, double rotation, double fw_noise, double rot_noise);

  Pose operator-(const Pose & p) const;
  Pose & operator=(const Pose & p);

  bool nearlyZero(void);

  double x_ = 0.0, y_ = 0.0, t_ = 0.0;

  uint16_t get16bitRepresentation(void);
  static uint16_t get16bitRepresentation(double);
};

}  // namespace emcl2

#endif  // EMCL2__POSE_HPP_
