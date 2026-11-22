// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <chrono>
#include <functional>
#include <optional>
#include <string>
#include "behaviortree_cpp_v3/control_node.h"
#include "bot_navigation/route_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"

namespace bot_navigation
{
std::optional<RemainingGeometry> remainingGeometry(
  const nav_msgs::msg::Path & path, double x, double y);
bool worthwhilePathChange(const RemainingGeometry &,const RemainingGeometry &,
  double yaw,double min_gain,double min_fraction,double mean_speed=1.3,double turn_rate=1.4);
// Children: current-path validity, live candidate planner, candidate validity.
// Optional candidate failure keeps the active path; mandatory failures retain
// the existing recovery semantics. Actual motion stays with the original RPP
// full-footprint, laser, limiter and collision-monitor protection chain.
class StablePathRefresh : public BT::ControlNode
{
public:
  using Clock=std::function<double()>;
  StablePathRefresh(const std::string &,const BT::NodeConfiguration &);
  StablePathRefresh(const std::string &,const BT::NodeConfiguration &,Clock);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;
  // The same parser is used by the ROS subscription and deterministic native
  // tests (including stale/wrong-path progress and malformed input).
  void receiveProgress(const std_msgs::msg::String &);
private:
  struct Progress {std::string key;RemainingGeometry geometry;double x,y,stamp,received;bool tracking;};
  std::optional<Progress> progress_;
  Clock clock_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::string path_key_;
  std::optional<double> last_refresh_,last_tick_;
  bool planning_{false},mandatory_{false};
  double requested_at_{0.},request_x_{0.},request_y_{0.};
};
}  // namespace bot_navigation
