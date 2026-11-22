// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "bot_navigation/stable_path_refresh.hpp"

namespace bot_navigation
{
// Use between the INITIAL plan and the FIRST FollowPath, never while an
// existing follower owns motion. A bounded opportunity to avoid committing to
// a large transient detour; this node never publishes a velocity command.
class InitialDetourWait : public BT::ControlNode
{
public:
  using Clock=StablePathRefresh::Clock;
  InitialDetourWait(const std::string &,const BT::NodeConfiguration &);
  InitialDetourWait(const std::string &,const BT::NodeConfiguration &,Clock);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;
private:
  Clock clock_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string goal_key_,done_goal_;
  std::optional<double> started_,last_attempt_;
  double start_x_{0.},start_y_{0.};
  bool planning_{false};
};
}  // namespace bot_navigation
