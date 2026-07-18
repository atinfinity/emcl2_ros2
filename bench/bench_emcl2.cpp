// Copyright 2026 emcl2_ros2 developers
// SPDX-FileCopyrightText: 2026 emcl2_ros2 developers
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
//
// Micro benchmark for the emcl2 hot paths. Build with -DBUILD_BENCHMARK=ON
// and run build/emcl2/bench_emcl2. Not executed on CI; timings depend on the
// machine, so compare numbers only across runs on the same host.

#ifdef _OPENMP
#include <omp.h>
#endif

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "emcl2/ExpResetMcl2.hpp"
#include "emcl2/LikelihoodFieldMap.hpp"
#include "emcl2/OdomModel.hpp"
#include "emcl2/Pose.hpp"
#include "emcl2/Scan.hpp"

namespace
{

template<typename F>
double benchMs(int rep, F && f)
{
  auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < rep; i++) {
    f();
  }
  auto t1 = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::milli>(t1 - t0).count() / rep;
}

}  // namespace

int main()
{
  // 20m x 20m map @ 0.05m: border walls + a wall block 2m east of the robot
  const int width = 400, height = 400;
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = 0.05;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(width * height, 0);
  for (int i = 0; i < width; i++) {
    grid.data[i] = 100;
    grid.data[i + (height - 1) * width] = 100;
    grid.data[0 + i * width] = 100;
    grid.data[(width - 1) + i * width] = 100;
  }
  for (int y = 190; y < 210; y++) {
    for (int x = 240; x < 244; x++) {
      grid.data[x + y * width] = 100;
    }
  }

  auto map = std::make_shared<emcl2::LikelihoodFieldMap>(grid, 0.2);
  auto om = std::make_shared<emcl2::OdomModel>(0.19, 0.0001, 0.13, 0.2);

  const int beams = 720;
  emcl2::Scan scan;
  scan.range_min_ = 0.1;
  scan.range_max_ = 10.0;
  scan.scan_increment_ = 1;
  scan.angle_min_ = -M_PI;
  scan.angle_max_ = M_PI;
  scan.angle_increment_ = 2.0 * M_PI / beams;
  for (int i = 0; i < beams; i++) {
    scan.ranges_.push_back(3.0);
    scan.directions_16bit_.push_back(
      emcl2::Pose::get16bitRepresentation(scan.angle_min_ + i * scan.angle_increment_));
  }

  const int num = 500;
  emcl2::ExpResetMcl2 pf(
    emcl2::Pose(10.0, 10.0, 0.0), num, scan, om, map, 0.5, 0.1, 0.2, 0.1, 0.1, false, false);

  // spread particles around the pose like a converged filter
  std::mt19937 rng(42);
  std::normal_distribution<double> pos(0.0, 0.1), ang(0.0, 0.05);
  for (auto & p : pf.particles_) {
    p.p_.x_ = 10.0 + pos(rng);
    p.p_.y_ = 10.0 + pos(rng);
    p.p_.t_ = ang(rng);
  }

  const int rep = 50;
  volatile double sink = 0.0;

  double t_likelihood = benchMs(
    rep, [&]() {
      double s = 0.0;
      for (auto & p : pf.particles_) {
        s += p.likelihood(map.get(), scan);
      }
      sink = s;
    });

  double t_wall = benchMs(
    rep, [&]() {
      int hits = 0;
      for (int i = 0; i < num; i += 10) {
        if (pf.particles_[i].wallConflict(map.get(), scan, 0.1, false)) {
          hits++;
        }
      }
      sink = hits;
    });

  auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  msg->angle_min = scan.angle_min_;
  msg->angle_max = scan.angle_max_;
  msg->angle_increment = scan.angle_increment_;
  msg->ranges.assign(beams, 3.0);

  double t_full = benchMs(
    rep, [&]() {
      pf.setScan(msg);  // new seq each time so sensorUpdate runs
      pf.sensorUpdate(0.0, 0.0, 0.0, false);
    });

#ifdef _OPENMP
  std::printf("OpenMP threads                  : %d\n", omp_get_max_threads());
#else
  std::printf("OpenMP                          : disabled\n");
#endif
  std::printf("=== emcl2 benchmark (%d particles, %d beams, %dx%d map) ===\n",
    num, beams, width, height);
  std::printf("likelihood loop (all particles) : %8.3f ms\n", t_likelihood);
  std::printf("wallConflict x50 (serial)       : %8.3f ms\n", t_wall);
  std::printf("full sensorUpdate               : %8.3f ms\n", t_full);
  return 0;
}
