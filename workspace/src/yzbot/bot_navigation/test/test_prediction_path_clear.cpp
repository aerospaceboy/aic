// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#include "bot_navigation/prediction_path_clear.hpp"
#include "gtest/gtest.h"
#include <chrono>
#include <thread>
#include <limits>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

nav_msgs::msg::Path path(const std::vector<std::pair<double, double>> & points)
{
  nav_msgs::msg::Path p;
  p.header.frame_id = "map";
  for (const auto & xy : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = xy.first;
    pose.pose.position.y = xy.second;
    p.poses.push_back(pose);
  }
  return p;
}
TEST(PredictionPathClear, ConflictAheadButNotBehind)
{
  const auto p = path({{-1, 0}, {0, 0}, {1, 0}, {2, 0}});
  EXPECT_TRUE(bot_navigation::predictionIntersectsPath(p, 0, 0, {{20, 0}}));
  EXPECT_FALSE(bot_navigation::predictionIntersectsPath(p, 0, 0, {{-20, 0}}));
  EXPECT_FALSE(bot_navigation::predictionIntersectsPath(p, 0, 0, {{20, 20}}));
}
TEST(PredictionPathClear, SparseSegmentsAndBoundedLookahead)
{
  const auto p = path({{0, 0}, {8, 0}});
  EXPECT_TRUE(bot_navigation::predictionIntersectsPath(p, 0, 0, {{39, 0}}));
  EXPECT_FALSE(bot_navigation::predictionIntersectsPath(p, 0, 0, {{100, 0}}));
  EXPECT_FALSE(bot_navigation::predictionIntersectsPath(p, 0, 0, {}));
  EXPECT_TRUE(bot_navigation::predictionIntersectsPath(p, 4, 0, {{100, 0}}));
  EXPECT_FALSE(bot_navigation::predictionIntersectsPath(p, 4, 0, {{39, 0}}));
  auto invalid = p;
  invalid.poses.back().pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(bot_navigation::predictionIntersectsPath(invalid, 0, 0, {{100, 0}}));
}

TEST(PredictionPathClear, RealSubscriptionReplacementValidationAndExpiry)
{
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = std::make_shared<rclcpp::Node>("test_prediction_condition");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "base_footprint";
  transform.header.stamp = node->now();
  transform.transform.rotation.w = 1;
  tf->setTransform(transform, "test", true);
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("node", node);
  config.blackboard->set("tf_buffer", tf);
  config.blackboard->set("path", path({{0,0},{2,0}}));
  config.input_ports["path"] = "{path}";
  config.input_ports["robot_base_frame"] = "base_footprint";
  bot_navigation::PredictionPathClear condition("check", config);
  auto publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/competition/dynamic_predictions", rclcpp::QoS(1));
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.height = 1;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(4, "x",1,sensor_msgs::msg::PointField::FLOAT32,
    "y",1,sensor_msgs::msg::PointField::FLOAT32, "z",1,sensor_msgs::msg::PointField::FLOAT32,
    "cost",1,sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> x(cloud,"x"), y(cloud,"y"), z(cloud,"z"), c(cloud,"cost");
  *x = 1.025; *y = .025; *z = .25; *c = 200;
  auto deliver = [&]() {
      cloud.header.stamp = node->now();
      for (int i = 0; i < 10; ++i) {
        publisher->publish(cloud);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        condition.tick();
      }
    };
  // Missing optional forecasts do not replace the independent physical check.
  EXPECT_EQ(condition.tick(), BT::NodeStatus::SUCCESS);
  deliver();
  EXPECT_EQ(condition.tick(), BT::NodeStatus::FAILURE);
  cloud.header.frame_id = "odom";
  modifier.resize(0);
  deliver();  // malformed/wrong-frame input must not clear the valid snapshot
  EXPECT_EQ(condition.tick(), BT::NodeStatus::FAILURE);
  cloud.header.frame_id = "map";
  deliver();  // valid replacing empty forecast retires old risk
  EXPECT_EQ(condition.tick(), BT::NodeStatus::SUCCESS);
  modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> rx(cloud,"x"), ry(cloud,"y"), rz(cloud,"z"), rc(cloud,"cost");
  *rx = 1.025; *ry = .025; *rz = .25; *rc = 200;
  deliver();
  EXPECT_EQ(condition.tick(), BT::NodeStatus::FAILURE);
  std::this_thread::sleep_for(std::chrono::milliseconds(550));
  EXPECT_EQ(condition.tick(), BT::NodeStatus::SUCCESS);
}
