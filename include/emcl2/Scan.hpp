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

#ifndef EMCL2__SCAN_HPP_
#define EMCL2__SCAN_HPP_

#include <iostream>
#include <vector>
#include <cstdint>

namespace emcl2
{

class Scan
{
public:
  int seq_ = 0;
  int scan_increment_ = 1;
  double angle_max_ = 0.0;
  double angle_min_ = 0.0;
  double angle_increment_ = 0.0;
  double range_max_ = 0.0;
  double range_min_ = 0.0;

  double lidar_pose_x_ = 0.0;
  double lidar_pose_y_ = 0.0;
  double lidar_pose_yaw_ = 0.0;

  std::vector<double> ranges_;
  std::vector<uint16_t> directions_16bit_;

  int countValidBeams(double * rate = NULL);
  bool valid(double range);
};

}  // namespace emcl2

#endif  // EMCL2__SCAN_HPP_
