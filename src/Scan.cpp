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

#include "emcl2/Scan.hpp"

#include <cmath>

namespace emcl2
{
Scan & Scan::operator=(const Scan & s)
{
  if (this == &s) {
    return *this;
  }

  seq_ = s.seq_;
  scan_increment_ = s.scan_increment_;
  angle_max_ = s.angle_max_;
  angle_min_ = s.angle_min_;
  angle_increment_ = s.angle_increment_;
  range_max_ = s.range_max_;
  range_min_ = s.range_min_;

        // It's not thread safe.
  ranges_.clear();
  copy(s.ranges_.begin(), s.ranges_.end(), back_inserter(ranges_));

  return *this;
}

int Scan::countValidBeams(double * rate)
{
  int ans = 0;
  for (size_t i = 0; i < ranges_.size(); i += scan_increment_) {
    if (valid(ranges_[i])) {
      ans++;
    }
  }

  if (rate != NULL) {
    *rate = static_cast<double>(ans) / ranges_.size() * scan_increment_;
  }

  return ans;
}

bool Scan::valid(double range)
{
  if (std::isnan(range) || std::isinf(range)) {
    return false;
  }

  return range_min_ <= range && range <= range_max_;
}

}  // namespace emcl2
