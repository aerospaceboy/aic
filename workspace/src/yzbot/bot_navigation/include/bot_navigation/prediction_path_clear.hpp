// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef BOT_NAVIGATION__PREDICTION_PATH_CLEAR_HPP_
#define BOT_NAVIGATION__PREDICTION_PATH_CLEAR_HPP_
#include <set>
#include <utility>
#include <memory>
#include <string>
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"

namespace bot_navigation
{
using RiskCells = std::set<std::pair<int, int>>;
// Inputs are already robot-configuration-space prediction cells, in map.
bool predictionIntersectsPath(
  const nav_msgs::msg::Path & path, double robot_x, double robot_y,
  const RiskCells & cells, double lookahead = 4.0);

class PredictionPathClear : public BT::ConditionNode
{
public:
  PredictionPathClear(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<std::string>("robot_base_frame", "base_footprint", "Robot frame")};
  }
  BT::NodeStatus tick() override;
private:
  void receive(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  RiskCells cells_;
  rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};
};
}  // namespace bot_navigation
#endif
