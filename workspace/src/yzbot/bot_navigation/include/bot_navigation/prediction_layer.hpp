// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef BOT_NAVIGATION__PREDICTION_LAYER_HPP_
#define BOT_NAVIGATION__PREDICTION_LAYER_HPP_

#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace bot_navigation
{
// Replacing, expiring forecasts. This layer can never clear static/raw layers.
class PredictionLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  void onInitialize() override;
  void updateBounds(double, double, double, double *, double *, double *, double *) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;
  void reset() override;
  bool isClearable() override {return true;}

private:
  void receive(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  struct Point {double x; double y; unsigned char cost;};
  std::mutex snapshot_mutex_;
  std::vector<Point> snapshot_;
  std::vector<Point> previous_;
  rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  double timeout_{0.5};
  double robot_egress_radius_{0.33};
  std::string source_frame_{"map"};
};
}  // namespace bot_navigation
#endif  // BOT_NAVIGATION__PREDICTION_LAYER_HPP_
