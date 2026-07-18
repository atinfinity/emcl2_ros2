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

#ifndef EMCL2__ODOMMODEL_HPP_
#define EMCL2__ODOMMODEL_HPP_

#include <random>

namespace emcl2
{
class OdomModel
{
public:
  OdomModel(double ff, double fr, double rf, double rr);
  void setDev(double length, double angle);
  double drawFwNoise(void);
  double drawRotNoise(void);

private:
  double fw_dev_;
  double rot_dev_;

  double fw_var_per_fw_;
  double fw_var_per_rot_;
  double rot_var_per_fw_;
  double rot_var_per_rot_;

  std::random_device seed_gen_;
  std::default_random_engine engine_ {seed_gen_()};

  std::normal_distribution< > std_norm_dist_;
};

}  // namespace emcl2

#endif  // EMCL2__ODOMMODEL_HPP_
