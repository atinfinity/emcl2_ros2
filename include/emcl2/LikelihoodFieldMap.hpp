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

#ifndef EMCL2__LIKELIHOODFIELDMAP_HPP_
#define EMCL2__LIKELIHOODFIELDMAP_HPP_

#include <utility>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "emcl2/Pose.hpp"
#include "emcl2/Scan.hpp"

namespace emcl2
{
class LikelihoodFieldMap
{
public:
  LikelihoodFieldMap(const nav_msgs::msg::OccupancyGrid & map, double likelihood_range);

  void setLikelihood(int x, int y, double range);
  uint8_t likelihood(double x, double y);

  std::vector<uint8_t> likelihoods_;
  int width_;
  int height_;

  double resolution_;
  double origin_x_;
  double origin_y_;

  void drawFreePoses(int num, std::vector<Pose> & result);

private:
  std::vector<std::pair<int, int>> free_cells_;
};

}  // namespace emcl2

#endif  // EMCL2__LIKELIHOODFIELDMAP_HPP_
