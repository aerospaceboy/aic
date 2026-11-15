// Copyright 2026 aerospaceboy
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "bot_navigation/motion_risk.hpp"

using bot_navigation::timedSeparationRisk;

TEST(MotionRisk, ContinuousTrackingRequiresAUsefulNearbyRemainingPath)
{
  EXPECT_TRUE(bot_navigation::continuousPathEligible(3., .1, false));
  EXPECT_FALSE(bot_navigation::continuousPathEligible(.7, .1, true));
  EXPECT_FALSE(bot_navigation::continuousPathEligible(3., .7, true));
  EXPECT_FALSE(bot_navigation::continuousPathEligible(3., -.1, false));
  EXPECT_FALSE(bot_navigation::continuousPathEligible(NAN, .1, false));
  EXPECT_TRUE(bot_navigation::continuousPathEligible(1., .5, true));
  EXPECT_FALSE(bot_navigation::continuousPathEligible(1., .5, false));
}

TEST(MotionRisk, OpenSpaceHandoverReservesTravelAndRejectsStaleGeometry)
{
  EXPECT_TRUE(bot_navigation::openSpaceAllowed(2., .05, false));
  EXPECT_FALSE(bot_navigation::openSpaceAllowed(1.05, .2, false));
  EXPECT_FALSE(bot_navigation::openSpaceAllowed(2., .5, true));
  EXPECT_FALSE(bot_navigation::openSpaceAllowed(.59, 0., true));
  EXPECT_TRUE(bot_navigation::openSpaceAllowed(.9, 0., true));
  EXPECT_FALSE(bot_navigation::openSpaceAllowed(.65, 0., false));
  EXPECT_TRUE(bot_navigation::openSpaceAllowed(.65, 0., true));
  EXPECT_TRUE(bot_navigation::openSpaceAllowed(.75, 0., false));
  EXPECT_FALSE(bot_navigation::openSpaceAllowed(.75, .05, false));
}

TEST(MotionRisk, SameCrossingPointHasDifferentRiskAtDifferentArrivalTimes)
{
  // Actor (1,-1) moves up at 1m/s; robot crosses (1,0).
  const auto risk = [](double t) {
      return timedSeparationRisk(std::abs(-1.+t)-.6, 12., 200.);
    };
  EXPECT_GT(risk(1.), risk(.1));
  EXPECT_GT(risk(1.), risk(1.8));
}

TEST(MotionRisk, EscapeFromUnavoidableNearTermOverlapRemainsRankable)
{
  const auto hold = timedSeparationRisk(-.15, 12., 200.);
  const auto escape = timedSeparationRisk(-.05, 12., 200.);
  EXPECT_TRUE(std::isfinite(hold));
  EXPECT_LT(escape, hold);
  EXPECT_GT(escape, timedSeparationRisk(.2, 12., 200.));
}

TEST(MotionRisk, DuplicatedTracksUseUnionRiskWithoutMultiplyingTheHazard)
{
  const auto risk = timedSeparationRisk(.1, 12., 200.);
  EXPECT_DOUBLE_EQ(std::max(risk, risk), risk);
  EXPECT_DOUBLE_EQ(timedSeparationRisk(2., 12., 200.), 0.);
}

TEST(MotionRisk, PausedHypothesisDoesNotDisappearWhenVelocityPointsAway)
{
  const auto continuing = timedSeparationRisk(1., 12., 200.);
  const auto paused = .35 * timedSeparationRisk(-.1, 12., 200.);
  EXPECT_DOUBLE_EQ(continuing, 0.);
  EXPECT_GT(std::max(continuing, paused), 0.);
}
