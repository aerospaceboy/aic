// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include "gtest/gtest.h"
#include "bot_navigation/pursuit_dynamics.hpp"

TEST(PursuitDynamics, LargeHeadingRecoveryHasHysteresisAcrossReplans) {
  bot_navigation::HeadingRecovery heading;
  EXPECT_FALSE(heading.update(.8));EXPECT_TRUE(heading.update(1.6));
  EXPECT_TRUE(heading.update(.9));EXPECT_TRUE(heading.update(-.36));
  EXPECT_FALSE(heading.update(-.34));EXPECT_FALSE(heading.update(.8));
  heading.update(2.);heading.reset();EXPECT_FALSE(heading.update(.8));
}
TEST(PursuitDynamics, ReversedSteeringBrakesRatherThanDrivingAway) {
  // Measured baseline: target +1.978 rad/s while physical yaw is -.838.
  EXPECT_DOUBLE_EQ(bot_navigation::pursuitTimeScale(1.978,-.838,4.,.05),0.);
  EXPECT_DOUBLE_EQ(bot_navigation::pursuitTimeScale(-1.978,.838,4.,.05),0.);
}
TEST(PursuitDynamics, AccelerationKeepsEveryPointOnTheSameCheckedCircle) {
  const double v=1.2,w=1.978,scale=bot_navigation::pursuitTimeScale(w,.3,4.,.05);
  ASSERT_GT(scale,0.);ASSERT_LT(scale,1.);
  EXPECT_NEAR((v*scale)/(w*scale),v/w,1.e-12);
  // The scaled one-second arc equals the original arc at scaled time.
  EXPECT_NEAR((v*scale)/(w*scale)*std::sin(w*scale),v/w*std::sin(w*scale),1.e-12);
  EXPECT_LE(std::abs(w*scale-.3),4.*.05+1.e-12);
}
TEST(PursuitDynamics, StraightTransitAndEstablishedCurvesKeepTheirSpeed) {
  EXPECT_DOUBLE_EQ(bot_navigation::pursuitTimeScale(0.,0.,4.,.05),1.);
  EXPECT_DOUBLE_EQ(bot_navigation::pursuitTimeScale(1.978,1.978,4.,.05),1.);
  EXPECT_DOUBLE_EQ(bot_navigation::pursuitTimeScale(.1,0.,4.,.05),1.);
}
TEST(PursuitDynamics, ExcessYawMustSettleBeforeStraightMotion) {
  EXPECT_DOUBLE_EQ(bot_navigation::pursuitTimeScale(0.,1.2,4.,.05),0.);
  EXPECT_DOUBLE_EQ(bot_navigation::pursuitTimeScale(.3,1.2,4.,.05),0.);
}
TEST(PursuitDynamics, InvalidFeedbackNeverCreatesACommand) {
  EXPECT_THROW(bot_navigation::pursuitTimeScale(NAN,0.,4.,.05),std::invalid_argument);
  EXPECT_THROW(bot_navigation::pursuitTimeScale(1.,0.,-1.,.05),std::invalid_argument);
  EXPECT_THROW(bot_navigation::HeadingRecovery(.3,.4),std::invalid_argument);
}
