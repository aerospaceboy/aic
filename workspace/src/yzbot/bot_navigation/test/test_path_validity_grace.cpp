// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#include <limits>
#include "gtest/gtest.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "bot_navigation/path_validity_grace.hpp"

class PathGraceTest : public ::testing::Test
{
protected:
  void SetUp() override {
    config.blackboard = BT::Blackboard::create();
    path.header.frame_id = "map"; path.header.stamp.sec = 1;
    geometry_msgs::msg::PoseStamped pose; pose.pose.position.x = 2.;
    path.poses.push_back(pose); config.blackboard->set("path", path);
    config.input_ports["path"] = "{path}"; config.input_ports["duration"] = "0.8";
    child = std::make_unique<BT::SimpleConditionNode>("physical_validity",
      [&](BT::TreeNode &) {return result;}, BT::NodeConfiguration{});
    grace = std::make_unique<bot_navigation::PathValidityGrace>("debounce", config, [&]() {return now;});
    grace->setChild(child.get());
  }
  BT::NodeConfiguration config; nav_msgs::msg::Path path;
  double now = 10.; BT::NodeStatus result = BT::NodeStatus::FAILURE;
  std::unique_ptr<BT::SimpleConditionNode> child;
  std::unique_ptr<bot_navigation::PathValidityGrace> grace;
};
TEST_F(PathGraceTest, ShortTransientDoesNotReplacePath) {
  EXPECT_EQ(grace->executeTick(), BT::NodeStatus::SUCCESS);
  now += .5; result = BT::NodeStatus::SUCCESS;
  EXPECT_EQ(grace->executeTick(), BT::NodeStatus::SUCCESS);
  now += 10.; result = BT::NodeStatus::FAILURE;
  EXPECT_EQ(grace->executeTick(), BT::NodeStatus::SUCCESS);
}
TEST_F(PathGraceTest, PersistentInvalidityExpiresEvenAcrossHalt) {
  EXPECT_EQ(grace->executeTick(), BT::NodeStatus::SUCCESS);
  grace->halt(); now += 1.;
  EXPECT_EQ(grace->executeTick(), BT::NodeStatus::FAILURE);
  grace->halt(); now += 1.;
  EXPECT_EQ(grace->executeTick(), BT::NodeStatus::FAILURE);
}
TEST_F(PathGraceTest, RecomputedPathToSameGoalStartsANewBoundedWindow) {
  grace->executeTick(); now += 1.;
  EXPECT_EQ(grace->executeTick(), BT::NodeStatus::FAILURE);
  path.header.stamp.sec++; config.blackboard->set("path", path);
  EXPECT_EQ(grace->executeTick(), BT::NodeStatus::SUCCESS);
  now += 1.; EXPECT_EQ(grace->executeTick(), BT::NodeStatus::FAILURE);
}
TEST_F(PathGraceTest, RunningValidityServiceIsNotFakedAsSuccess) {
  result = BT::NodeStatus::RUNNING;
  EXPECT_EQ(grace->executeTick(), BT::NodeStatus::RUNNING);
}
TEST_F(PathGraceTest, MalformedPathAndClockFailImmediately) {
  path.poses[0].pose.position.x = std::numeric_limits<double>::quiet_NaN();
  config.blackboard->set("path", path);
  EXPECT_EQ(grace->executeTick(), BT::NodeStatus::FAILURE);
  path.poses[0].pose.position.x = 2.; config.blackboard->set("path", path);
  grace->executeTick(); now -= 1.;
  EXPECT_EQ(grace->executeTick(), BT::NodeStatus::FAILURE);
}
TEST_F(PathGraceTest, ExcessiveGraceCannotDisableReplanning) {
  config.input_ports["duration"] = "20";
  bot_navigation::PathValidityGrace invalid("invalid", config, [&]() {return now;});
  invalid.setChild(child.get());
  EXPECT_EQ(invalid.executeTick(), BT::NodeStatus::FAILURE);
}
#ifdef PATH_GRACE_PLUGIN
TEST_F(PathGraceTest, InstalledPluginBuildsARealBehaviorTree) {
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(PATH_GRACE_PLUGIN);
  factory.registerSimpleCondition("InvalidPhysicalPath", [](BT::TreeNode &) {
    return BT::NodeStatus::FAILURE;
  });
  auto tree = factory.createTreeFromText(
    "<root main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\">"
    "<PathValidityGrace path=\"{path}\" duration=\"0.8\">"
    "<InvalidPhysicalPath/></PathValidityGrace></BehaviorTree></root>", config.blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  tree.haltTree();
}
#endif
