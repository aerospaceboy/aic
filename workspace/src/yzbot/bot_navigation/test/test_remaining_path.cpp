// Copyright 2026 aerospaceboy
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "bot_navigation/remaining_path.hpp"
#include "bot_navigation/controller_fallback.hpp"

TEST(RemainingPath, HandoverAfterLongNominalLegKeepsCurrentSegmentAndGoal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i < 401; ++i) {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = i * .05;
    path.poses.push_back(p);
  }
  auto remaining = bot_navigation::remainingPath(path, 300);
  EXPECT_EQ(remaining.header.frame_id, "map");
  EXPECT_NEAR(remaining.poses.front().pose.position.x, 14.95, 1.e-6);
  EXPECT_DOUBLE_EQ(remaining.poses.back().pose.position.x, 20.);
  EXPECT_EQ(path.poses.size(), 401u);
  // Re-entry must use the newly projected progress, rather than the suffix
  // retained by an MPPI child that has been inactive for another ten metres.
  remaining = bot_navigation::remainingPath(path, 399);
  EXPECT_EQ(remaining.poses.size(), 3u);
}

TEST(RemainingPath, EmptyAndBoundaryPlansAreBounded)
{
  nav_msgs::msg::Path path;
  EXPECT_TRUE(bot_navigation::remainingPath(path, 100).poses.empty());
  path.poses.resize(2);
  EXPECT_EQ(bot_navigation::remainingPath(path, 0).poses.size(), 2u);
  EXPECT_EQ(bot_navigation::remainingPath(path, 100).poses.size(), 2u);
}

TEST(ControllerFallback, CurrentValidatedCommandSurvivesOptimizerFailure)
{
  bool dynamic = true;
  std::string failure;
  auto failing = []() -> int {throw std::runtime_error("optimizer failed");};
  EXPECT_EQ(bot_navigation::dynamicWithFallback(failing, 42, nullptr, dynamic, failure), 42);
  EXPECT_FALSE(dynamic);
  EXPECT_EQ(failure, "optimizer failed");
}

TEST(ControllerFallback, BothInvalidControllersPropagateFailure)
{
  bool dynamic = true;
  std::string failure;
  std::exception_ptr rejected;
  try {throw std::runtime_error("current nominal collision");}
  catch (...) {rejected = std::current_exception();}
  auto failing = []() -> int {throw std::runtime_error("optimizer failed");};
  EXPECT_THROW(bot_navigation::dynamicWithFallback(failing, 42, rejected, dynamic, failure),
    std::runtime_error);
  EXPECT_TRUE(dynamic);
}

TEST(ControllerFallback, ValidOptimizerDoesNotUseNominal)
{
  bool dynamic = true;
  std::string failure;
  EXPECT_EQ(bot_navigation::dynamicWithFallback([] {return 7;}, 42, nullptr, dynamic, failure), 7);
  EXPECT_TRUE(dynamic);
  EXPECT_TRUE(failure.empty());
}

TEST(ControllerFallback, StageCapSurvivesOptimizerResetAndPreservesCurvature)
{
  geometry_msgs::msg::Twist command;
  command.linear.x = 1.78; command.linear.y = .4; command.angular.z = 1.2;
  bot_navigation::capTranslation(command, .75);
  EXPECT_NEAR(std::hypot(command.linear.x, command.linear.y), .75, 1.e-9);
  EXPECT_NEAR(command.linear.y / command.linear.x, .4 / 1.78, 1.e-9);
  EXPECT_NEAR(command.angular.z / command.linear.x, 1.2 / 1.78, 1.e-9);
  const auto before = command;
  bot_navigation::capTranslation(command, 0.);
  EXPECT_EQ(command, before);
}

TEST(ControllerFallback, TranslationCapDoesNotSuppressIntentionalInPlaceTurn)
{
  geometry_msgs::msg::Twist command;
  command.angular.z=1.2;
  bot_navigation::capTranslation(command,.75);
  EXPECT_DOUBLE_EQ(command.angular.z,1.2);
}

TEST(CollisionPreview, HealthyPrimaryNeverCallsSecondary)
{
  bool used=false;std::string reason;int calls=0;
  EXPECT_EQ(bot_navigation::collisionPreview<int>([]{return 7;},[&]{++calls;return 8;},
    true,1.7,.6,used,reason),7);
  EXPECT_EQ(calls,0);EXPECT_FALSE(used);
}
TEST(CollisionPreview, ABrakedCollisionCanUseAFreshValidatedShortArc)
{
  bool used=false;std::string reason;
  auto primary=[]()->int{throw std::runtime_error("Collision ahead");};
  EXPECT_EQ(bot_navigation::collisionPreview<int>(primary,[]{return 8;},
    true,.3,.6,used,reason),8);
  EXPECT_TRUE(used);EXPECT_EQ(reason,"Collision ahead");
}
TEST(CollisionPreview, SpeedAndErrorClassificationNeverBypassPrimaryFailure)
{
  for(const auto & error:{"TF unavailable","empty path","Collision ahead"}) {
    for(double speed:{.1,1.7}) {
    bool used=false;std::string reason;int calls=0;
    auto primary=[&]()->int{throw std::runtime_error(error);};
    if(speed<.65 && std::string(error)=="Collision ahead")continue;
    EXPECT_THROW(bot_navigation::collisionPreview<int>(primary,[&]{++calls;return 8;},
      true,speed,.6,used,reason),std::runtime_error);
    EXPECT_EQ(calls,0);EXPECT_FALSE(used);
    }
  }
}
TEST(CollisionPreview, TwoUnsafeArcsStillPropagateCollision)
{
  bool used=false;std::string reason;
  auto unsafe=[]()->int{throw std::runtime_error("Collision ahead");};
  EXPECT_THROW(bot_navigation::collisionPreview<int>(unsafe,unsafe,
    true,.1,.6,used,reason),std::runtime_error);
  EXPECT_FALSE(used);
}
