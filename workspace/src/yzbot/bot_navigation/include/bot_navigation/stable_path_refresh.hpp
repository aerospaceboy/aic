// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <chrono>
#include <cmath>
#include <functional>
#include <optional>
#include <string>
#include "behaviortree_cpp_v3/control_node.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace bot_navigation
{
struct RemainingGeometry {double length{0.}, heading{0.};};
std::optional<RemainingGeometry> remainingGeometry(
  const nav_msgs::msg::Path & path, double x, double y);
bool worthwhilePathChange(const RemainingGeometry & old_path,
  const RemainingGeometry & candidate, double yaw, double min_gain,
  double min_fraction, double mean_speed = 1.3, double turn_rate = 1.4);

// Two children: validity condition, then the unchanged live-map planner action
// writing a SEPARATE candidate. Optional failures preserve the old valid path;
// mandatory failures still propagate. No motion or collision permissions here.
class StablePathRefresh : public BT::ControlNode
{
public:
  using Clock = std::function<double()>;
  StablePathRefresh(const std::string &, const BT::NodeConfiguration &);
  StablePathRefresh(const std::string &, const BT::NodeConfiguration &, Clock);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
  void halt() override;
private:
  Clock clock_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string path_key_;
  std::optional<double> last_refresh_, last_tick_;
  bool planning_{false}, mandatory_{false};
};
}  // namespace bot_navigation
