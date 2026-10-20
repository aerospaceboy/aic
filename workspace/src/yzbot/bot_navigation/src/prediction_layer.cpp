// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#include "bot_navigation/prediction_layer.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bot_navigation
{
void PredictionLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {throw std::runtime_error("Prediction layer has no lifecycle node");}
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("timeout", rclcpp::ParameterValue(0.5));
  declareParameter("source_frame", rclcpp::ParameterValue("map"));
  declareParameter("robot_egress_radius", rclcpp::ParameterValue(0.33));
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".timeout", timeout_);
  node->get_parameter(name_ + ".source_frame", source_frame_);
  node->get_parameter(name_ + ".robot_egress_radius", robot_egress_radius_);
  if (source_frame_.empty()) {throw std::runtime_error("Prediction source frame is empty");}
  if (timeout_ <= 0.0 || timeout_ > 1.0) {
    throw std::runtime_error("Prediction timeout must be in (0, 1] seconds");
  }
  if (robot_egress_radius_ < 0.28 || robot_egress_radius_ > 0.40) {
    throw std::runtime_error("Prediction robot egress radius must be in [0.28, 0.40]");
  }
  default_value_ = nav2_costmap_2d::NO_INFORMATION;
  matchSize();
  current_ = true;
  subscription_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/competition/dynamic_predictions", rclcpp::QoS(1),
    std::bind(&PredictionLayer::receive, this, std::placeholders::_1));
}

void PredictionLayer::receive(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  // Forecasts already use the authoritative map TF. Never guess another frame
  // or accept malformed/oversized data as a valid replacing snapshot.
  if (message->header.frame_id != source_frame_ ||
    message->is_bigendian || message->height != 1 || message->point_step != 16 ||
    message->data.size() != static_cast<size_t>(message->width) * 16 ||
    message->row_step != message->width * 16 || message->fields.size() != 4 ||
    message->width > 20000)
  {
    return;
  }
  const char * names[] = {"x", "y", "z", "cost"};
  for (size_t i = 0; i < 4; ++i) {
    const auto & field = message->fields[i];
    if (field.name != names[i] || field.offset != i * 4 || field.count != 1 ||
      field.datatype != sensor_msgs::msg::PointField::FLOAT32) {return;}
  }
  const rclcpp::Time stamp(message->header.stamp, clock_->get_clock_type());
  const double age = (clock_->now() - stamp).seconds();
  if (age < -0.05 || age > timeout_) {return;}
  geometry_msgs::msg::TransformStamped transform;
  const bool needs_transform = source_frame_ != layered_costmap_->getGlobalFrameID();
  if (needs_transform) {
    if (!tf_) {return;}
    try {
      transform = tf_->lookupTransform(
        layered_costmap_->getGlobalFrameID(), source_frame_, stamp, rclcpp::Duration(0, 0));
    } catch (const tf2::TransformException &) {return;}
  }
  std::vector<Point> points;
  try {
    if (message->width > 0) {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x"), y(*message, "y"),
        z(*message, "z"), cost(*message, "cost");
      for (; x != x.end(); ++x, ++y, ++z, ++cost) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
        if (*cost != 200.0f && *cost != 254.0f) {return;}
        if (needs_transform) {
          geometry_msgs::msg::PointStamped input, output;
          input.point.x = *x; input.point.y = *y; input.point.z = *z;
          tf2::doTransform(input, output, transform);
          points.push_back({output.point.x, output.point.y, static_cast<unsigned char>(*cost)});
        } else {points.push_back({*x, *y, static_cast<unsigned char>(*cost)});}
      }
    }
  } catch (const std::runtime_error &) {return;}
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  if (stamp < stamp_) {return;}
  stamp_ = stamp;
  snapshot_ = std::move(points);
}

void PredictionLayer::updateBounds(
  double robot_x, double robot_y, double,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (layered_costmap_->isRolling()) {
    const auto master = layered_costmap_->getCostmap();
    updateOrigin(master->getOriginX(), master->getOriginY());
  }
  std::vector<Point> points;
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    const double age = (clock_->now() - stamp_).seconds();
    if (enabled_ && age >= 0.0 && age <= timeout_) {points = snapshot_;}
  }
  // Retire only our own previous body cells. Updating the old AND new bounds
  // lets inflation recompute the vacated gap without clearing any other layer.
  resetMaps();
  for (const auto & point : previous_) {touch(point.x, point.y, min_x, min_y, max_x, max_y);}
  for (const auto & point : points) {
    // Prediction is an advisory envelope around a separately sensed physical
    // obstacle. Never let that envelope cover the robot's current start cell:
    // NavFn cannot plan any egress from INSCRIBED/LETHAL space. Raw lidar,
    // the collision monitor and the actor proximity guard remain unchanged.
    // Clear only hard current-body samples; soft future risk remains visible.
    if (point.cost == nav2_costmap_2d::LETHAL_OBSTACLE &&
      std::hypot(point.x - robot_x, point.y - robot_y) <= robot_egress_radius_)
    {
      touch(point.x, point.y, min_x, min_y, max_x, max_y);
      continue;
    }
    unsigned int mx, my;
    if (worldToMap(point.x, point.y, mx, my)) {
      const auto prior = getCost(mx, my);
      setCost(mx, my, prior == nav2_costmap_2d::NO_INFORMATION ? point.cost :
        std::max(prior, point.cost));
      touch(point.x, point.y, min_x, min_y, max_x, max_y);
    }
  }
  previous_ = std::move(points);
  current_ = true;  // raw obstacle layer and collision monitor own sensor health
}

void PredictionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j)
{
  updateWithMax(master, min_i, min_j, max_i, max_j);
}

void PredictionLayer::reset()
{
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  snapshot_.clear();
  // Keep previous_ until updateBounds reports its old bounds for retirement.
}
}  // namespace bot_navigation

PLUGINLIB_EXPORT_CLASS(bot_navigation::PredictionLayer, nav2_costmap_2d::Layer)
