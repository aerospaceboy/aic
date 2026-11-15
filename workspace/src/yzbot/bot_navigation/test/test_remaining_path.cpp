// Copyright 2026 aerospaceboy
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "bot_navigation/remaining_path.hpp"

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
