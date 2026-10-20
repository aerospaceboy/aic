// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#include "bot_navigation/prediction_path_clear.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <functional>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace bot_navigation
{
bool predictionIntersectsPath(
  const nav_msgs::msg::Path & path, double robot_x, double robot_y,
  const RiskCells & cells, double lookahead)
{
  if (path.poses.empty() || cells.empty()) {return false;}
  if (!std::isfinite(robot_x) || !std::isfinite(robot_y)) {return true;}
  for (const auto & pose : path.poses) {
    if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y)) {
      return true;
    }
  }
  size_t start = 0;
  auto a = path.poses.front().pose.position;
  double best = std::numeric_limits<double>::max();
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto p = path.poses[i - 1].pose.position;
    const auto q = path.poses[i].pose.position;
    const double dx = q.x - p.x, dy = q.y - p.y, squared = dx * dx + dy * dy;
    const double ratio = squared > 0 ? std::max(0.0, std::min(1.0,
      ((robot_x - p.x) * dx + (robot_y - p.y) * dy) / squared)) : 0;
    const double x = p.x + ratio * dx, y = p.y + ratio * dy;
    const double distance = std::hypot(x - robot_x, y - robot_y);
    if (distance < best) {start = i; best = distance; a.x = x; a.y = y;}
  }
  double travelled = 0;
  for (size_t i = start; i < path.poses.size(); ++i) {
    const auto b = path.poses[i].pose.position;
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    const size_t count = std::max(size_t(1), static_cast<size_t>(std::ceil(length / .025)));
    for (size_t j = 0; j <= count; ++j) {
      const double ratio = static_cast<double>(j) / count;
      if (travelled + length * ratio > lookahead) {return false;}
      const int x = static_cast<int>(std::floor((a.x + ratio * (b.x - a.x)) / .05));
      const int y = static_cast<int>(std::floor((a.y + ratio * (b.y - a.y)) / .05));
      if (cells.count({x, y})) {return true;}
    }
    travelled += length;
    a = b;
  }
  return false;
}

PredictionPathClear::PredictionPathClear(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  // The Nav2 BT client node is also spun by its service/action plugins.
  // Own only this subscription's callback group; never add the whole node
  // to a second executor or block waiting for a forecast.
  group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  rclcpp::SubscriptionOptions options;
  options.callback_group = group_;
  subscription_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/competition/dynamic_predictions", rclcpp::QoS(1),
    std::bind(&PredictionPathClear::receive, this, std::placeholders::_1), options);
  executor_.add_callback_group(group_, node_->get_node_base_interface());
  stamp_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
}

void PredictionPathClear::receive(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  if (message->header.frame_id != "map" || message->is_bigendian || message->height != 1 ||
    message->point_step != 16 || message->row_step != message->width * 16 ||
    message->width > 20000 || message->fields.size() != 4 ||
    message->data.size() != static_cast<size_t>(message->width) * 16) {return;}
  const char * names[] = {"x", "y", "z", "cost"};
  for (size_t i = 0; i < 4; ++i) {
    const auto & field = message->fields[i];
    if (field.name != names[i] || field.offset != i * 4 || field.count != 1 ||
      field.datatype != sensor_msgs::msg::PointField::FLOAT32) {return;}
  }
  const rclcpp::Time stamp(message->header.stamp, node_->get_clock()->get_clock_type());
  const double age = (node_->now() - stamp).seconds();
  if (stamp < stamp_ || age < -.05 || age > .5) {return;}
  RiskCells next;
  if (message->width) {
    sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x"), y(*message, "y"),
      z(*message, "z"), cost(*message, "cost");
    for (; x != x.end(); ++x, ++y, ++z, ++cost) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z) ||
        (*cost != 200.f && *cost != 254.f) || std::abs(*x) > 10000 ||
        std::abs(*y) > 10000) {return;}
      next.insert({static_cast<int>(std::floor(*x / .05)),
        static_cast<int>(std::floor(*y / .05))});
    }
  }
  cells_ = std::move(next);
  stamp_ = stamp;
}

BT::NodeStatus PredictionPathClear::tick()
{
  executor_.spin_some(std::chrono::milliseconds(2));
  nav_msgs::msg::Path path;
  if (!getInput("path", path) || path.header.frame_id != "map" || path.poses.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  // This optional prediction check does not replace IsPathValid, raw scans,
  // physical footprint checks or the final guard. Expired risk is not a wall.
  const double age = (node_->now() - stamp_).seconds();
  if (age < 0 || age > .5 || cells_.empty()) {return BT::NodeStatus::SUCCESS;}
  std::string frame;
  getInput("robot_base_frame", frame);
  try {
    const auto pose = tf_->lookupTransform("map", frame, tf2::TimePointZero);
    if (predictionIntersectsPath(path, pose.transform.translation.x,
      pose.transform.translation.y, cells_))
    {
      RCLCPP_DEBUG(node_->get_logger(), "prediction_path_conflict: requesting dynamic replan");
      return BT::NodeStatus::FAILURE;
    }
  } catch (const tf2::TransformException &) {return BT::NodeStatus::FAILURE;}
  return BT::NodeStatus::SUCCESS;
}
}  // namespace bot_navigation

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bot_navigation::PredictionPathClear>("PredictionPathClear");
}
