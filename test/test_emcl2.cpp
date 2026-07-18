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

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "emcl2/ExpResetMcl2.hpp"
#include "emcl2/LikelihoodFieldMap.hpp"
#include "emcl2/OdomModel.hpp"
#include "emcl2/Pose.hpp"
#include "emcl2/Scan.hpp"

namespace
{

nav_msgs::msg::OccupancyGrid makeMap(int width, int height, double resolution)
{
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = width;
  map.info.height = height;
  map.info.resolution = resolution;
  map.info.origin.position.x = 0.0;
  map.info.origin.position.y = 0.0;
  map.info.origin.orientation.w = 1.0;
  map.data.assign(width * height, 0);
  return map;
}

emcl2::Scan makeScan(void)
{
  emcl2::Scan scan;
  scan.range_min_ = 0.1;
  scan.range_max_ = 10.0;
  scan.scan_increment_ = 1;
  return scan;
}

std::shared_ptr<emcl2::ExpResetMcl2> makePf(int num, const emcl2::Pose & init_pose)
{
  auto map = std::make_shared<emcl2::LikelihoodFieldMap>(makeMap(10, 10, 0.1), 0.2);
  auto om = std::make_shared<emcl2::OdomModel>(0.1, 0.1, 0.1, 0.1);
  return std::make_shared<emcl2::ExpResetMcl2>(
    init_pose, num, makeScan(), om, map, 0.5, 0.1, 0.2, 0.1, 0.1, false, true);
}

}  // namespace

TEST(ScanTest, validRejectsBadRanges)
{
  emcl2::Scan scan = makeScan();

  EXPECT_TRUE(scan.valid(1.0));
  EXPECT_TRUE(scan.valid(0.1));
  EXPECT_TRUE(scan.valid(10.0));
  EXPECT_FALSE(scan.valid(0.05));
  EXPECT_FALSE(scan.valid(11.0));
  EXPECT_FALSE(scan.valid(NAN));
  EXPECT_FALSE(scan.valid(INFINITY));
  EXPECT_FALSE(scan.valid(-INFINITY));
}

TEST(ScanTest, countValidBeams)
{
  emcl2::Scan scan = makeScan();
  scan.ranges_ = {1.0, NAN, 2.0, 20.0, 3.0, 0.05};

  EXPECT_EQ(scan.countValidBeams(), 3);

  scan.scan_increment_ = 2;  // checks indices 0, 2, 4 -> all valid
  EXPECT_EQ(scan.countValidBeams(), 3);
}

TEST(PoseTest, defaultConstructorInitializesMembers)
{
  emcl2::Pose p;
  EXPECT_EQ(p.x_, 0.0);
  EXPECT_EQ(p.y_, 0.0);
  EXPECT_EQ(p.t_, 0.0);
}

TEST(PoseTest, get16bitRepresentation)
{
  EXPECT_EQ(emcl2::Pose::get16bitRepresentation(0.0), 0);
  EXPECT_EQ(emcl2::Pose::get16bitRepresentation(M_PI / 2), 1 << 14);
  EXPECT_EQ(emcl2::Pose::get16bitRepresentation(M_PI), 1 << 15);
  EXPECT_EQ(emcl2::Pose::get16bitRepresentation(-M_PI), 1 << 15);
  EXPECT_EQ(emcl2::Pose::get16bitRepresentation(2 * M_PI), 0);
}

TEST(PoseTest, subtractionNormalizesAngle)
{
  emcl2::Pose a(1.0, 2.0, 3.0);
  emcl2::Pose b(0.5, 0.5, -3.0);
  emcl2::Pose d = a - b;

  EXPECT_DOUBLE_EQ(d.x_, 0.5);
  EXPECT_DOUBLE_EQ(d.y_, 1.5);
  EXPECT_NEAR(d.t_, 6.0 - 2 * M_PI, 1e-9);
}

TEST(LikelihoodFieldMapTest, likelihoodFieldAroundWall)
{
  auto grid = makeMap(10, 10, 0.1);
  grid.data[5 + 5 * 10] = 100;  // wall at cell (5, 5)
  emcl2::LikelihoodFieldMap map(grid, 0.2);

  EXPECT_EQ(map.likelihood(0.55, 0.55), 255);  // on the wall
  EXPECT_EQ(map.likelihood(0.55, 0.65), 127);  // one cell away
  EXPECT_EQ(map.likelihood(0.55, 0.75), 0);    // two cells away (weight 0)
  EXPECT_EQ(map.likelihood(-0.5, -0.5), 0);    // out of the map
  EXPECT_EQ(map.likelihood(5.0, 5.0), 0);      // out of the map
}

TEST(LikelihoodFieldMapTest, zeroLikelihoodRangeDoesNotProduceNan)
{
  auto grid = makeMap(10, 10, 0.1);
  grid.data[5 + 5 * 10] = 100;
  emcl2::LikelihoodFieldMap map(grid, 0.0);

  EXPECT_EQ(map.likelihood(0.55, 0.55), 255);
}

TEST(LikelihoodFieldMapTest, drawFreePosesStaysInsideMap)
{
  emcl2::LikelihoodFieldMap map(makeMap(10, 10, 0.1), 0.2);

  std::vector<emcl2::Pose> poses;
  map.drawFreePoses(20, poses);

  ASSERT_EQ(poses.size(), 20u);
  for (const auto & p : poses) {
    EXPECT_GE(p.x_, 0.0);
    EXPECT_LE(p.x_, 1.0);
    EXPECT_GE(p.y_, 0.0);
    EXPECT_LE(p.y_, 1.0);
    EXPECT_GE(p.t_, -M_PI);
    EXPECT_LE(p.t_, M_PI);
  }
}

TEST(MclTest, invalidParticleNumberThrows)
{
  EXPECT_THROW(makePf(0, emcl2::Pose(0.0, 0.0, 0.0)), std::invalid_argument);
  EXPECT_THROW(makePf(-1, emcl2::Pose(0.0, 0.0, 0.0)), std::invalid_argument);
}

TEST(MclTest, meanPoseOfIdenticalParticles)
{
  auto pf = makePf(10, emcl2::Pose(0.3, 0.4, 0.5));

  double x, y, t, xd, yd, td, xy, yt, tx;
  pf->meanPose(x, y, t, xd, yd, td, xy, yt, tx);

  EXPECT_DOUBLE_EQ(x, 0.3);
  EXPECT_DOUBLE_EQ(y, 0.4);
  EXPECT_DOUBLE_EQ(t, 0.5);
  EXPECT_NEAR(xd, 0.0, 1e-12);
  EXPECT_NEAR(yd, 0.0, 1e-12);
  EXPECT_NEAR(td, 0.0, 1e-12);
}

TEST(MclTest, meanPoseWithSingleParticleHasNoNan)
{
  auto pf = makePf(1, emcl2::Pose(0.3, 0.4, 0.5));

  double x, y, t, xd, yd, td, xy, yt, tx;
  pf->meanPose(x, y, t, xd, yd, td, xy, yt, tx);

  EXPECT_FALSE(std::isnan(xd));
  EXPECT_FALSE(std::isnan(yd));
  EXPECT_FALSE(std::isnan(td));
  EXPECT_FALSE(std::isnan(xy));
  EXPECT_FALSE(std::isnan(yt));
  EXPECT_FALSE(std::isnan(tx));
}

TEST(MclTest, meanPoseHandlesAngleWraparound)
{
  auto pf = makePf(2, emcl2::Pose(0.0, 0.0, 0.0));
  pf->particles_[0].p_.t_ = M_PI - 0.05;
  pf->particles_[1].p_.t_ = -M_PI + 0.05;

  double x, y, t, xd, yd, td, xy, yt, tx;
  pf->meanPose(x, y, t, xd, yd, td, xy, yt, tx);

  EXPECT_NEAR(std::fabs(t), M_PI, 0.06);
}

TEST(MclTest, initializeResetsParticles)
{
  auto pf = makePf(10, emcl2::Pose(0.3, 0.4, 0.5));
  pf->initialize(0.7, 0.8, -0.5);

  for (const auto & p : pf->particles_) {
    EXPECT_DOUBLE_EQ(p.p_.x_, 0.7);
    EXPECT_DOUBLE_EQ(p.p_.y_, 0.8);
    EXPECT_DOUBLE_EQ(p.p_.t_, -0.5);
    EXPECT_DOUBLE_EQ(p.w_, 0.1);
  }
}

// The MemoryLifecycleTest cases exercise construction, copy and destruction
// paths intensively. They act as functional tests in a normal build and as
// leak checks when built with -DENABLE_SANITIZER=ON (AddressSanitizer with
// LeakSanitizer).

TEST(MemoryLifecycleTest, likelihoodFieldMapConstructDestroyLoop)
{
  auto grid = makeMap(100, 100, 0.05);
  for (int x = 0; x < 100; x++) {
    grid.data[x] = 100;  // wall along the bottom row
  }

  for (int i = 0; i < 100; i++) {
    emcl2::LikelihoodFieldMap map(grid, 0.2);
    EXPECT_EQ(map.likelihood(0.025, 0.025), 255);
  }
}

TEST(MemoryLifecycleTest, likelihoodFieldMapCopyIsSafe)
{
  auto grid = makeMap(10, 10, 0.1);
  grid.data[5 + 5 * 10] = 100;
  emcl2::LikelihoodFieldMap original(grid, 0.2);

  emcl2::LikelihoodFieldMap copy(original);
  emcl2::LikelihoodFieldMap assigned(makeMap(10, 10, 0.1), 0.2);
  assigned = original;

  EXPECT_EQ(copy.likelihood(0.55, 0.55), 255);
  EXPECT_EQ(assigned.likelihood(0.55, 0.55), 255);
}

TEST(MemoryLifecycleTest, particleFilterLifecycleLoop)
{
  for (int i = 0; i < 20; i++) {
    auto pf = makePf(50, emcl2::Pose(0.3, 0.4, 0.5));
    // exercise the map replacement path used when a map is re-received
    pf->setMap(std::make_shared<emcl2::LikelihoodFieldMap>(makeMap(10, 10, 0.1), 0.2));
    pf->motionUpdate(0.1 * i, 0.0, 0.0);
    pf->motionUpdate(0.1 * i + 0.05, 0.02, 0.1);
    pf->initialize(0.5, 0.5, 0.0);
    pf->simpleReset();
  }
}

TEST(MclTest, simpleResetScattersParticlesInsideMap)
{
  auto pf = makePf(10, emcl2::Pose(0.3, 0.4, 0.5));
  pf->simpleReset();

  for (const auto & p : pf->particles_) {
    EXPECT_GE(p.p_.x_, 0.0);
    EXPECT_LE(p.p_.x_, 1.0);
    EXPECT_GE(p.p_.y_, 0.0);
    EXPECT_LE(p.p_.y_, 1.0);
    EXPECT_DOUBLE_EQ(p.w_, 0.1);
  }
}
