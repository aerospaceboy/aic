// Copyright 2026 aerospaceboy
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "bot_navigation/patrol_step.hpp"
using namespace bot_navigation;

TEST(PatrolStep, StoppedRobotWithinOldFreezeRadiusDoesNotStopPatrol)
{
  auto step=guardedPatrolStep({0,0},{2,0},.7,-1,.01,{{{0,0},.58}});
  EXPECT_FALSE(step.blocked);EXPECT_FALSE(step.reversed);
  EXPECT_NEAR(step.phase,.69,1.e-9);
}
TEST(PatrolStep, ClosingActorReversesRatherThanWaitsForRobot)
{
  auto step=guardedPatrolStep({0,0},{2,0},.585,-1,.01,{{{0,0},.58}});
  EXPECT_FALSE(step.blocked);EXPECT_TRUE(step.reversed);
  EXPECT_NEAR(step.phase,.595,1.e-9);
}
TEST(PatrolStep, StationaryRobotLeavesNoLongFreezeAndNoPositionJump)
{
  double phase=1.;int direction=-1;int blocked=0,reversed=0;
  for(int i=0;i<1500;++i) {
    auto before=patrolPoint({0,0},{2,0},phase);
    auto step=guardedPatrolStep({0,0},{2,0},phase,direction,.01,{{{0,0},.58}});
    auto after=patrolPoint({0,0},{2,0},step.phase);
    EXPECT_GE(after.x,.58-1.e-9);
    EXPECT_LE(std::hypot(after.x-before.x,after.y-before.y),.01+1.e-9);
    phase=step.phase;direction=step.direction;
    blocked+=step.blocked;reversed+=step.reversed;
  }
  EXPECT_EQ(blocked,0);EXPECT_GT(reversed,1);
}
TEST(PatrolStep, ExistingOverlapAllowsOnlySeparation)
{
  auto step=guardedPatrolStep({0,0},{2,0},.4,-1,.01,{{{0,0},.58}});
  EXPECT_TRUE(step.reversed);EXPECT_GT(step.phase,.4);
}
TEST(PatrolStep, TwoPhysicalBlockersStillCannotBeCrossed)
{
  auto step=guardedPatrolStep({0,0},{2,0},1.,1,.01,
    {{{.5,0},.505},{{1.5,0},.505}});
  EXPECT_TRUE(step.blocked);EXPECT_DOUBLE_EQ(step.phase,1.);
}
TEST(PatrolStep, FoldedEndpointCannotHideAnUnsafeExcursion)
{
  auto step=guardedPatrolStep({0,0},{2,0},1.995,1,.01,{{{2.58,0},.585}});
  EXPECT_TRUE(step.reversed);EXPECT_FALSE(step.blocked);
}
