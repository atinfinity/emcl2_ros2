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

#include "emcl2/Pose.hpp"

#include <cmath>
#include <sstream>

namespace emcl2
{
Pose::Pose(double x, double y, double t) {set(x, y, t);}

Pose::Pose(const Pose & other) {set(other);}

void Pose::set(double x, double y, double t)
{
  x_ = x;
  y_ = y;
  t_ = t;
}

void Pose::set(const Pose & p)
{
  x_ = p.x_;
  y_ = p.y_;
  t_ = p.t_;
}

std::string Pose::to_s(void)
{
  std::stringstream s;
  s << "x:" << x_ << "\ty:" << y_ << "\tt:" << t_;
  return s.str();
}

void Pose::normalizeAngle(void)
{
  while (t_ > M_PI) {
    t_ -= 2 * M_PI;
  }
  while (t_ < -M_PI) {
    t_ += 2 * M_PI;
  }
}

Pose Pose::operator-(const Pose & p) const
{
  Pose ans(x_ - p.x_, y_ - p.y_, t_ - p.t_);
  ans.normalizeAngle();

  return ans;
}

Pose Pose::operator=(const Pose & p)
{
  if (this != &p) {
    x_ = p.x_;
    y_ = p.y_;
    t_ = p.t_;
  }
  return *this;
}

void Pose::move(double length, double direction, double rotation, double fw_noise, double rot_noise)
{
  x_ += (length + fw_noise) * cos(direction + rot_noise + t_);
  y_ += (length + fw_noise) * sin(direction + rot_noise + t_);
  t_ += rotation + rot_noise;
  normalizeAngle();
}

bool Pose::nearlyZero(void) {return fabs(x_) < 0.001 && fabs(y_) < 0.001 && fabs(t_) < 0.001;}

uint16_t Pose::get16bitRepresentation(void)
{
  int tmp = t_ / M_PI * (1 << 15);
  while (tmp < 0) {
    tmp += (1 << 16);
  }
  while (tmp >= (1 << 16)) {
    tmp -= (1 << 16);
  }

  return (uint16_t)tmp;
}

uint16_t Pose::get16bitRepresentation(double t)
{
  int tmp = t / M_PI * (1 << 15);
  while (tmp < 0) {
    tmp += (1 << 16);
  }
  while (tmp >= (1 << 16)) {
    tmp -= (1 << 16);
  }

  return (uint16_t)tmp;
}

}  // namespace emcl2
