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
#include "emcl2/Mcl.hpp"

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace emcl2
{
Mcl::Mcl(
  const Pose & p, int num, const Scan & scan, const std::shared_ptr<OdomModel> & odom_model,
  const std::shared_ptr<LikelihoodFieldMap> & map)
{
  odom_model_ = odom_model;
  map_ = map;
  scan_ = scan;

  if (num <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("emcl2_node"), "NO PARTICLE");
    throw std::invalid_argument("num_particles must be positive");
  }

  Particle particle(p.x_, p.y_, p.t_, 1.0 / num);
  for (int i = 0; i < num; i++) {
    particles_.push_back(particle);
  }

  processed_seq_ = -1;
  alpha_ = 1.0;

  for (int i = 0; i < (1 << 16); i++) {
    cos_[i] = cos(M_PI * i / (1 << 15));
    sin_[i] = sin(M_PI * i / (1 << 15));
  }
}

Mcl::~Mcl() {}

void Mcl::resampling(void)
{
  std::vector<double> accum;
  accum.push_back(particles_[0].w_);
  for (size_t i = 1; i < particles_.size(); i++) {
    accum.push_back(accum.back() + particles_[i].w_);
  }

  std::vector<Particle> old(particles_);

  std::uniform_real_distribution<double> ud(0.0, 1.0);
  double start = ud(rng_) / particles_.size();
  double step = 1.0 / particles_.size();

  std::vector<int> chosen;

  size_t tick = 0;
  for (size_t i = 0; i < particles_.size(); i++) {
    while (accum[tick] <= start + i * step) {
      tick++;
      if (tick == particles_.size()) {
        // floating point rounding can push the target beyond accum.back()
        tick = particles_.size() - 1;
        break;
      }
    }
    chosen.push_back(tick);
  }

  for (size_t i = 0; i < particles_.size(); i++) {
    particles_[i] = old[chosen[i]];
  }
}

void Mcl::motionUpdate(double x, double y, double t)
{
  if (!last_odom_) {
    last_odom_ = std::make_unique<Pose>(x, y, t);
    prev_odom_ = std::make_unique<Pose>(x, y, t);
    return;
  } else {
    last_odom_->set(x, y, t);
  }

  Pose d = *last_odom_ - *prev_odom_;
  if (d.nearlyZero()) {
    return;
  }

  double fw_length = sqrt(d.x_ * d.x_ + d.y_ * d.y_);
  double fw_direction = atan2(d.y_, d.x_) - prev_odom_->t_;

  odom_model_->setDev(fw_length, d.t_);

  for (auto & p : particles_) {
    p.p_.move(
                  fw_length, fw_direction, d.t_, odom_model_->drawFwNoise(),
                  odom_model_->drawRotNoise());
  }

  prev_odom_->set(*last_odom_);
}

void Mcl::largestClusterParticles(std::vector<const Particle *> & out)
{
  const double res = 0.5;           // clustering grid resolution [m]
  const double merge_radius = 1.0;  // gather radius around the peak cell [m]

  // Bin particles by position and find the densest cell. After resampling the
  // particle density already reflects the posterior weight, so the densest cell
  // is the dominant mode.
  std::map<std::pair<int, int>, int> counts;
  std::pair<int, int> peak_cell;
  int peak_count = -1;
  for (const auto & p : particles_) {
    std::pair<int, int> cell(
      static_cast<int>(std::floor(p.p_.x_ / res)),
      static_cast<int>(std::floor(p.p_.y_ / res)));
    int c = ++counts[cell];
    if (c > peak_count) {
      peak_count = c;
      peak_cell = cell;
    }
  }

  // Centroid of the densest cell, then gather every particle within a radius of
  // it so a mode that straddles a cell boundary is captured as one cluster.
  double cx = 0.0, cy = 0.0;
  int n = 0;
  for (const auto & p : particles_) {
    if (static_cast<int>(std::floor(p.p_.x_ / res)) == peak_cell.first &&
      static_cast<int>(std::floor(p.p_.y_ / res)) == peak_cell.second)
    {
      cx += p.p_.x_;
      cy += p.p_.y_;
      n++;
    }
  }
  if (n > 0) {
    cx /= n;
    cy /= n;
  }

  out.clear();
  const double r2 = merge_radius * merge_radius;
  for (const auto & p : particles_) {
    const double dx = p.p_.x_ - cx;
    const double dy = p.p_.y_ - cy;
    if (dx * dx + dy * dy <= r2) {
      out.push_back(&p);
    }
  }
  if (out.empty()) {  // safety: never estimate from an empty set
    for (const auto & p : particles_) {
      out.push_back(&p);
    }
  }
}

void Mcl::meanPose(
  double & x_mean, double & y_mean, double & t_mean, double & x_dev, double & y_dev, double & t_dev,
  double & xy_cov, double & yt_cov, double & tx_cov)
{
  std::vector<const Particle *> ps;
  if (estimate_largest_cluster_) {
    largestClusterParticles(ps);
  } else {
    ps.reserve(particles_.size());
    for (const auto & p : particles_) {
      ps.push_back(&p);
    }
  }
  const size_t num = ps.size();

  double x, y, t, t2;
  x = y = t = t2 = 0.0;
  for (const auto * p : ps) {
    x += p->p_.x_;
    y += p->p_.y_;
    t += p->p_.t_;
    t2 += normalizeAngle(p->p_.t_ + M_PI);
  }

  x_mean = x / num;
  y_mean = y / num;
  t_mean = t / num;
  double t2_mean = t2 / num;

  double xx, yy, tt, tt2;
  xx = yy = tt = tt2 = 0.0;
  for (const auto * p : ps) {
    xx += pow(p->p_.x_ - x_mean, 2);
    yy += pow(p->p_.y_ - y_mean, 2);
    tt += pow(normalizeAngle(p->p_.t_ - t_mean), 2);
    tt2 += pow(normalizeAngle(p->p_.t_ + M_PI) - t2_mean, 2);
  }

  if (tt > tt2) {
    tt = tt2;
    t_mean = normalizeAngle(t2_mean - M_PI);
  }

  size_t denom = num > 1 ? num - 1 : 1;
  x_dev = xx / denom;
  y_dev = yy / denom;
  t_dev = tt / denom;

  double xy, yt, tx;
  xy = yt = tx = 0.0;
  for (const auto * p : ps) {
    xy += (p->p_.x_ - x_mean) * (p->p_.y_ - y_mean);
    yt += (p->p_.y_ - y_mean) * (normalizeAngle(p->p_.t_ - t_mean));
    tx += (p->p_.x_ - x_mean) * (normalizeAngle(p->p_.t_ - t_mean));
  }

  xy_cov = xy / denom;
  yt_cov = yt / denom;
  tx_cov = tx / denom;
}

double Mcl::normalizeAngle(double t)
{
  while (t > M_PI) {
    t -= 2 * M_PI;
  }
  while (t < -M_PI) {
    t += 2 * M_PI;
  }

  return t;
}

void Mcl::setScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  if (msg->ranges.size() != scan_.ranges_.size()) {
    scan_.ranges_.resize(msg->ranges.size());
  }

  scan_.seq_++;
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    scan_.ranges_[i] = msg->ranges[i];
  }

  scan_.angle_min_ = msg->angle_min;
  scan_.angle_max_ = msg->angle_max;
  scan_.angle_increment_ = msg->angle_increment;
}

void Mcl::setMap(const std::shared_ptr<LikelihoodFieldMap> & map) {map_ = map;}

double Mcl::normalizeBelief(void)
{
  double sum = 0.0;
  for (const auto & p : particles_) {
    sum += p.w_;
  }

  if (sum < 0.000000000001) {
    return sum;
  }

  for (auto & p : particles_) {
    p.w_ /= sum;
  }

  return sum;
}

void Mcl::resetWeight(void)
{
  for (auto & p : particles_) {
    p.w_ = 1.0 / particles_.size();
  }
}

void Mcl::initialize(double x, double y, double t)
{
  Pose new_pose(x, y, t);
  for (auto & p : particles_) {
    p.p_ = new_pose;
  }

  resetWeight();
}

void Mcl::simpleReset(void)
{
  std::vector<Pose> poses;
  map_->drawFreePoses(particles_.size(), poses);

  for (size_t i = 0; i < poses.size(); i++) {
    particles_[i].p_ = poses[i];
    particles_[i].w_ = 1.0 / particles_.size();
  }
}

double Mcl::cos_[(1 << 16)];
double Mcl::sin_[(1 << 16)];

}  // namespace emcl2
