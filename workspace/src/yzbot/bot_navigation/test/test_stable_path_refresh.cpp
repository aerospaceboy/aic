// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#include "gtest/gtest.h"
#include <nlohmann/json.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "bot_navigation/stable_path_refresh.hpp"

namespace
{
nav_msgs::msg::Path route(std::initializer_list<std::pair<double,double>> xy, int stamp=1) {
  nav_msgs::msg::Path p; p.header.frame_id="map"; p.header.stamp.sec=stamp;
  for(const auto & v:xy) {
    geometry_msgs::msg::PoseStamped q; q.pose.position.x=v.first; q.pose.position.y=v.second;
    q.pose.orientation.w=1.; p.poses.push_back(q);
  }
  return p;
}
class Planner : public BT::StatefulActionNode {
public:
  Planner(std::function<BT::NodeStatus()> fn) : BT::StatefulActionNode("live_planner",{}), fn_(fn) {}
  BT::NodeStatus onStart() override {++requests; return fn_();}
  BT::NodeStatus onRunning() override {return fn_();}
  void onHalted() override {++cancels;}
  int requests{0},cancels{0};
private: std::function<BT::NodeStatus()> fn_;
};
}
TEST(StableRouteGeometry, PassedSegmentsDoNotCreateFakeGains) {
  auto g=bot_navigation::suffixGeometry(route({{5,2},{5,5},{10,5}}),5.,2.);
  ASSERT_TRUE(g); EXPECT_NEAR(g->length,8.,1.e-9);
  EXPECT_NEAR(g->heading,std::acos(-1.)/2,1.e-9);
  auto point=bot_navigation::remainingGeometry(route({{1,1}}),1,1);
  ASSERT_TRUE(point); EXPECT_DOUBLE_EQ(point->length,0.);
}
TEST(StableRouteGeometry, SavingsMustPayForAnExtraUTurn) {
  using bot_navigation::worthwhilePathChange; using bot_navigation::RemainingGeometry;
  const RemainingGeometry old{20.,0.};
  EXPECT_FALSE(worthwhilePathChange(old,{17.5,std::acos(-1.)},0.,1.5,.12));
  EXPECT_TRUE(worthwhilePathChange(old,{10.,std::acos(-1.)},0.,1.5,.12));
  EXPECT_FALSE(worthwhilePathChange(old,{19.,0.},0.,1.5,.12));
}
class StableRefreshTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {rclcpp::init(0,nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {
    node=std::make_shared<rclcpp::Node>("stable_path_native");
    tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    geometry_msgs::msg::TransformStamped t;t.header.frame_id="map";
    t.child_frame_id="base_footprint";t.transform.rotation.w=1.;t.header.stamp=node->now();
    ASSERT_TRUE(tf->setTransform(t,"native",false));
    config.blackboard=BT::Blackboard::create(); config.blackboard->set("node",node);
    config.blackboard->set("tf_buffer",tf);
    original=route({{0,0},{0,8},{10,8},{10,0},{20,0}});
    candidate=route({{0,0},{20,0}},2);config.blackboard->set("path",original);
    config.input_ports={{"path","{path}"},{"candidate","{candidate}"},{"interval","3.0"},
      {"min_gain","1.5"},{"min_fraction","0.12"},{"robot_base_frame","base_footprint"}};
    config.output_ports["path"]="{path}";
    valid=std::make_unique<BT::SimpleConditionNode>("valid",[&](BT::TreeNode &) {return validity;},BT::NodeConfiguration{});
    planner=std::make_unique<Planner>([&]() {
      if (plan_status==BT::NodeStatus::SUCCESS)config.blackboard->set("candidate",candidate);
      return plan_status;
    });
    refresh=std::make_unique<bot_navigation::StablePathRefresh>("refresh",config,[&](){return now;});
    candidate_valid=std::make_unique<BT::SimpleConditionNode>("candidate_valid",
      [&](BT::TreeNode &) {return candidate_validity;},BT::NodeConfiguration{});
    refresh->addChild(valid.get());refresh->addChild(planner.get());refresh->addChild(candidate_valid.get());
    progress();
  }
  void progress(const std::string & key="",bool tracking=true) {
    const auto geometry=bot_navigation::suffixGeometry(original,0.,0.);
    nlohmann::json j={{"path_key",key.empty()?bot_navigation::routeKey(original):key},
      {"remaining_m",geometry->length},{"heading",geometry->heading},{"x",0.},{"y",0.},
      {"stamp",node->now().seconds()},{"tracking",tracking}};
    std_msgs::msg::String msg;msg.data=j.dump();refresh->receiveProgress(msg);
  }
  void advance(double dt) {now+=dt;progress();}
  nav_msgs::msg::Path current() {return config.blackboard->get<nav_msgs::msg::Path>("path");}
  rclcpp::Node::SharedPtr node;std::shared_ptr<tf2_ros::Buffer> tf;BT::NodeConfiguration config;
  nav_msgs::msg::Path original,candidate;double now{10.};
  BT::NodeStatus candidate_validity{BT::NodeStatus::SUCCESS};
  BT::NodeStatus validity{BT::NodeStatus::SUCCESS},plan_status{BT::NodeStatus::SUCCESS};
  std::unique_ptr<BT::SimpleConditionNode> valid,candidate_valid;std::unique_ptr<Planner> planner;
  std::unique_ptr<bot_navigation::StablePathRefresh> refresh;
};
TEST_F(StableRefreshTest, AcceptsShorteningAndPreservesPathDuringAsyncPlanning) {
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::SUCCESS); EXPECT_EQ(planner->requests,0);
  advance(3.);plan_status=BT::NodeStatus::RUNNING;
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::RUNNING);
  EXPECT_EQ(current().poses.size(),original.poses.size());
  plan_status=BT::NodeStatus::SUCCESS;advance(.02);
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(current().poses.size(),candidate.poses.size());
  refresh->halt();now+=.1;refresh->executeTick();EXPECT_EQ(planner->requests,1);
}
TEST_F(StableRefreshTest, OptionalFailureKeepsPathButMandatoryFailurePropagates) {
  refresh->executeTick();advance(3.);plan_status=BT::NodeStatus::FAILURE;
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(current().header.stamp.sec,1);
  validity=BT::NodeStatus::FAILURE;
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::FAILURE);
}
TEST_F(StableRefreshTest, MandatoryRepairAcceptsLongerSafeRouteImmediately) {
  validity=BT::NodeStatus::FAILURE;candidate=route({{0,0},{0,20},{20,20},{20,0}},3);
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(current().header.stamp.sec,3);EXPECT_EQ(planner->requests,1);
}
TEST_F(StableRefreshTest, RejectsChangedGoalAndMinorJitter) {
  refresh->executeTick();advance(3.);candidate=route({{0,0},{20,1}},2);
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::SUCCESS);EXPECT_EQ(current().header.stamp.sec,1);
  advance(3.);candidate=route({{0,0},{0,7.5},{10,7.5},{10,0},{20,0}},2);
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::SUCCESS);EXPECT_EQ(current().header.stamp.sec,1);
}
TEST_F(StableRefreshTest, CancellationCannotInstallUnfinishedCandidate) {
  refresh->executeTick();advance(3.);plan_status=BT::NodeStatus::RUNNING;
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::RUNNING);refresh->halt();
  EXPECT_EQ(planner->cancels,1);EXPECT_EQ(current().header.stamp.sec,1);
}
TEST_F(StableRefreshTest, RealPluginFactoryBuildsControlAndBidirectionalPorts) {
  BT::BehaviorTreeFactory factory;factory.registerFromPlugin(PATH_GRACE_PLUGIN);
  factory.registerSimpleCondition("Invalid",[](BT::TreeNode &){return BT::NodeStatus::FAILURE;});
  factory.registerSimpleAction("Compute",[&](BT::TreeNode &){config.blackboard->set("candidate",candidate);return BT::NodeStatus::SUCCESS;});
  auto tree=factory.createTreeFromText("<root main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\">"
    "<StablePathRefresh path=\"{path}\" candidate=\"{candidate}\"><Invalid/><Compute/>"
    "<Invalid/></StablePathRefresh></BehaviorTree></root>",config.blackboard);
  EXPECT_EQ(tree.tickRoot(),BT::NodeStatus::SUCCESS);EXPECT_EQ(current().header.stamp.sec,2);
}

TEST(StableRouteGeometry, NearbyReturnLegCannotEraseTheUntraversedLoop) {
  const auto path=route({{0,0},{0,10},{.25,10},{.25,0},{8,0}});
  auto suffix=bot_navigation::suffixGeometry(path,.2,0.);
  ASSERT_TRUE(suffix);EXPECT_NEAR(suffix->length,28.2,1.e-9);
  // A candidate is only projected into its first 1.5 m, matching the follower's
  // forward window, never onto the nearby return segment across a wall.
  auto bounded=bot_navigation::remainingGeometry(path,.2,0.);
  ASSERT_TRUE(bounded);EXPECT_NEAR(bounded->length,28.2,1.e-9);
}
TEST(StableRouteGeometry, PlanIdentityIncludesIntermediateGeometry) {
  auto a=route({{0,0},{0,10},{10,0}});auto b=route({{0,0},{10,10},{10,0}});
  EXPECT_NE(bot_navigation::routeKey(a),bot_navigation::routeKey(b));
  EXPECT_EQ(bot_navigation::routeKey(a),bot_navigation::routeKey(a));
}
TEST_F(StableRefreshTest, WrongOrExpiredControllerProgressCannotTriggerOptionalPlanning) {
  refresh->executeTick();now+=3.;progress("different-plan");
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::SUCCESS);EXPECT_EQ(planner->requests,0);
  now+=3.;progress();now+=.6;
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::SUCCESS);EXPECT_EQ(planner->requests,0);
}
TEST_F(StableRefreshTest, FreshPlannerResultStillNeedsCurrentMapValidity) {
  refresh->executeTick();advance(3.);candidate_validity=BT::NodeStatus::FAILURE;
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(current().header.stamp.sec,1);EXPECT_EQ(planner->requests,1);
}
TEST_F(StableRefreshTest, OptionalPlannerCannotBlockTrackingIndefinitely) {
  refresh->executeTick();advance(3.);plan_status=BT::NodeStatus::RUNNING;
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::RUNNING);
  advance(1.01);EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(planner->cancels,1);EXPECT_EQ(current().header.stamp.sec,1);
}
TEST_F(StableRefreshTest, ExpiredProgressWhilePlanningCannotInstallCandidate) {
  refresh->executeTick();advance(3.);plan_status=BT::NodeStatus::RUNNING;
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::RUNNING);
  now+=.6;plan_status=BT::NodeStatus::SUCCESS;
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(current().header.stamp.sec,1);
}
TEST_F(StableRefreshTest, ChangedActivePathCancelsInFlightOptionalCandidate) {
  refresh->executeTick();advance(3.);plan_status=BT::NodeStatus::RUNNING;
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::RUNNING);
  auto replaced=route({{0,0},{0,30},{20,0}},4);config.blackboard->set("path",replaced);
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(planner->cancels,1);EXPECT_EQ(current().header.stamp.sec,4);
}

TEST_F(StableRefreshTest, InitialHeadingRecoveryDoesNotHideAnEarlyShortRoute) {
  refresh->executeTick();advance(3.);progress("",false);
  EXPECT_EQ(refresh->executeTick(),BT::NodeStatus::SUCCESS);
  EXPECT_EQ(planner->requests,1);EXPECT_EQ(current().header.stamp.sec,2);
}
