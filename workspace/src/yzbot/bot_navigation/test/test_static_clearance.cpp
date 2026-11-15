// Copyright 2026 aerospaceboy
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <limits>
#include "bot_navigation/static_clearance.hpp"

using bot_navigation::StaticClearanceGrid;
TEST(StaticClearance, FutureWallApproachFailsAlthoughCurrentPoseIsOpen)
{
  std::vector<int8_t> cells(10000,0);
  for(size_t y=0;y<100;++y)cells[y*100+50]=100;
  StaticClearanceGrid grid;grid.build(cells,100,100,.05,0.,0.,0.,.48);
  EXPECT_TRUE(grid.clear(1.,2.));
  EXPECT_FALSE(grid.clear(2.2,2.));
  EXPECT_TRUE(grid.clear(1.9,2.));
}
TEST(StaticClearance, UnknownOutsideAndMissingGeometryFailClosed)
{
  StaticClearanceGrid grid;EXPECT_FALSE(grid.clear(1.,1.));
  std::vector<int8_t> cells(10000,0);cells[50*100+50]=-1;
  grid.build(cells,100,100,.05,0.,0.,0.,.48);
  EXPECT_FALSE(grid.clear(2.5,2.5));
  EXPECT_FALSE(grid.clear(-.1,1.));
  EXPECT_FALSE(grid.clear(std::numeric_limits<double>::quiet_NaN(),1.));
}
TEST(StaticClearance, RotatedMapOriginIsRespected)
{
  std::vector<int8_t> cells(10000,0);cells[50*100+50]=100;
  StaticClearanceGrid grid;grid.build(cells,100,100,.05,10.,-2.,M_PI/2,.48);
  EXPECT_FALSE(grid.clear(7.5,.5));
  EXPECT_TRUE(grid.clear(9.,-1.));
}
TEST(StaticClearance, InvalidMapOrUndersizedLoadedEnvelopeIsRejected)
{
  StaticClearanceGrid grid;
  EXPECT_THROW(grid.build({0},10,10,.05,0.,0.,0.,.48),std::invalid_argument);
  EXPECT_THROW(grid.build({0},1,1,.05,0.,0.,0.,.28),std::invalid_argument);
}
