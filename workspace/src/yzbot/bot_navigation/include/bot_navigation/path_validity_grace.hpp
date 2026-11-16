// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <chrono>
#include <cmath>
#include <functional>
#include <optional>
#include <string>
#include "behaviortree_cpp_v3/decorator_node.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bot_navigation
{
// Debounce GLOBAL replanning only. This node never issues motion or replaces
// the 20 Hz RPP footprint check, laser guard, or final collision monitor.
class PathValidityGrace : public BT::DecoratorNode
{
public:
  using Clock = std::function<double()>;
  PathValidityGrace(const std::string & name, const BT::NodeConfiguration & config)
  : PathValidityGrace(name, config, []() {
      return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    }) {}

  PathValidityGrace(const std::string & name, const BT::NodeConfiguration & config, Clock clock)
  : BT::DecoratorNode(name, config), clock_(std::move(clock)) {
    config.blackboard->get("node", node_);
  }
  static BT::PortsList providedPorts() {
    return {BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<double>("duration", .8, "Bounded global replanning grace, seconds")};
  }
  BT::NodeStatus tick() override {
    nav_msgs::msg::Path path;
    double duration = 0.;
    if (!getInput("path", path) || path.header.frame_id.empty() || path.poses.empty() ||
      !getInput("duration", duration) || !std::isfinite(duration) || duration < 0. || duration > 2.)
      return BT::NodeStatus::FAILURE;
    for (const auto & pose : path.poses) {
      const auto & p = pose.pose.position;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
        return BT::NodeStatus::FAILURE;
    }
    const auto & last = path.poses.back().pose.position;
    const auto & first = path.poses.front().pose.position;
    const auto key = path.header.frame_id + ":" +
      std::to_string(path.header.stamp.sec) + ":" + std::to_string(path.header.stamp.nanosec) +
      ":" + std::to_string(first.x) + ":" + std::to_string(first.y) +
      ":" + std::to_string(last.x) + ":" + std::to_string(last.y);
    if (key != path_key_) {invalid_since_.reset(); path_key_ = key;}
    const double now = clock_();
    if (!std::isfinite(now) || (last_tick_ && now < *last_tick_)) {
      invalid_since_.reset(); last_tick_ = now; return BT::NodeStatus::FAILURE;
    }
    last_tick_ = now;
    const auto status = child_node_->executeTick();
    if (status == BT::NodeStatus::RUNNING) {return status;}
    resetChild();
    if (status == BT::NodeStatus::SUCCESS) {invalid_since_.reset(); return status;}
    if (status != BT::NodeStatus::FAILURE) {return BT::NodeStatus::FAILURE;}
    if (!invalid_since_) {
      invalid_since_ = now;
      if (node_ && duration > 0.)
        RCLCPP_INFO(node_->get_logger(), "path_replan_debounce grace_sec=%.2f collision_checks=active", duration);
    }
    return now - *invalid_since_ < duration ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  // Deliberately retain the invalid interval across ordinary BT halts/resets.
  // A new path (including same-goal replans) or a valid sample resets it.
private:
  Clock clock_;
  std::string path_key_;
  std::optional<double> invalid_since_, last_tick_;
  rclcpp::Node::SharedPtr node_;
};
}  // namespace bot_navigation
