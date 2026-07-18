// Copyright 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
// SPDX-License-Identifier: BSD-3-Clause
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Ryuichi Ueda nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "emcl2/LikelihoodFieldMap.hpp"

#include <algorithm>
#include <random>

#include "emcl2/Pose.hpp"

namespace emcl2
{
LikelihoodFieldMap::LikelihoodFieldMap(
  const nav_msgs::msg::OccupancyGrid & map, double likelihood_range)
{
  width_ = map.info.width;
  height_ = map.info.height;

  origin_x_ = map.info.origin.position.x;
  origin_y_ = map.info.origin.position.y;

  resolution_ = map.info.resolution;

  for (int x = 0; x < width_; x++) {
    likelihoods_.push_back(new uint8_t[height_]);

    for (int y = 0; y < height_; y++) {
      likelihoods_[x][y] = 0;
    }
  }

  for (int x = 0; x < width_; x++) {
    for (int y = 0; y < height_; y++) {
      int v = map.data[x + y * width_];
      if (v > 50) {
        setLikelihood(x, y, likelihood_range);
      } else if (0 <= v && v <= 50) {
        free_cells_.push_back(std::pair<int, int>(x, y));
      }
    }
  }

  // normalize();
}

LikelihoodFieldMap::~LikelihoodFieldMap()
{
  for (auto & e : likelihoods_) {
    delete[] e;
  }
}

uint8_t LikelihoodFieldMap::likelihood(double x, double y)
{
  int ix = static_cast<int>(floor((x - origin_x_) / resolution_));
  int iy = static_cast<int>(floor((y - origin_y_) / resolution_));

  if (ix < 0 || iy < 0 || ix >= width_ || iy >= height_) {
    return 0.0;
  }

  return likelihoods_[ix][iy];
}

void LikelihoodFieldMap::setLikelihood(int x, int y, double range)
{
  int cell_num = static_cast<int>(ceil(range / resolution_));
  std::vector<uint8_t> weights;
  for (int i = 0; i <= cell_num; i++) {
    weights.push_back(static_cast<int>(255 * (1.0 - static_cast<double>(i) / cell_num)));
  }

  for (int i = -cell_num; i <= cell_num; i++) {
    for (int j = -cell_num; j <= cell_num; j++) {
      if (i + x >= 0 && j + y >= 0 && i + x < width_ && j + y < height_) {
        likelihoods_[i + x][j + y] = std::max(
                                  likelihoods_[i + x][j + y],
                                  std::min(weights[abs(i)], weights[abs(j)]));
      }
    }
  }
}

/*
void LikelihoodFieldMap::normalize(void)
{
	uint8_t maximum = 0;
	for (int x = 0; x < width_; x++) {
		for (int y = 0; y < height_; y++) {
			maximum = std::max(likelihoods_[x][y], maximum);
		}
	}

	for (int x = 0; x < width_; x++) {
		for (int y = 0; y < height_; y++) {
			likelihoods_[x][y] /= maximum;
		}
	}
}
*/

void LikelihoodFieldMap::drawFreePoses(int num, std::vector<Pose> & result)
{
  std::random_device seed_gen;
  std::mt19937 engine{seed_gen()};
  std::vector<std::pair<int, int>> chosen_cells;

  sample(free_cells_.begin(), free_cells_.end(), back_inserter(chosen_cells), num, engine);

  std::uniform_real_distribution<double> offset(0.0, resolution_);
  std::uniform_real_distribution<double> angle(-M_PI, M_PI);
  for (auto & c : chosen_cells) {
    Pose p;
    p.x_ = c.first * resolution_ + offset(engine) + origin_x_;
    p.y_ = c.second * resolution_ + offset(engine) + origin_y_;
    p.t_ = angle(engine);
    result.push_back(p);
  }
}

}  // namespace emcl2
