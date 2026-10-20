// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#include <chrono>
#include <limits>
#include <memory>
#include <thread>
#include "gtest/gtest.h"
#include "bot_navigation/prediction_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class PredictionTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    lifecycle = std::make_shared<nav2_util::LifecycleNode>("prediction_unit", "");
    sender = std::make_shared<rclcpp::Node>("prediction_sender");
    buffer = std::make_shared<tf2_ros::Buffer>(lifecycle->get_clock());
    parent = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    parent->resizeMap(20, 20, .1, 0, 0);
    layer = std::make_unique<bot_navigation::PredictionLayer>();
    group = lifecycle->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    layer->initialize(parent.get(), "prediction", buffer.get(), lifecycle, group);
    publisher = sender->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/competition/dynamic_predictions", rclcpp::QoS(1));
    executor.add_node(lifecycle->get_node_base_interface());
    executor.add_node(sender);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (publisher->get_subscription_count() == 0 &&
      std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(publisher->get_subscription_count(), 1u);
  }
  void send(double x, double y, bool empty = false, const char * frame = "map", float cost = 254)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame;
    cloud.header.stamp = lifecycle->now();
    cloud.height = 1;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "cost", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(empty ? 0 : 1);
    if (!empty) {
      sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z"),
        icost(cloud, "cost");
      *ix = x; *iy = y; *iz = .25; *icost = cost;
    }
    publisher->publish(cloud);
    for (int i = 0; i < 5; ++i) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  void bounds()
  {
    min_x = min_y = std::numeric_limits<double>::infinity();
    max_x = max_y = -std::numeric_limits<double>::infinity();
    layer->updateBounds(0, 0, 0, &min_x, &min_y, &max_x, &max_y);
  }
  void merge()
  {
    layer->updateCosts(*parent->getCostmap(), 0, 0, 20, 20);
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  std::shared_ptr<nav2_util::LifecycleNode> lifecycle;
  rclcpp::Node::SharedPtr sender;
  std::shared_ptr<tf2_ros::Buffer> buffer;
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> parent;
  std::unique_ptr<bot_navigation::PredictionLayer> layer;
  double min_x, min_y, max_x, max_y;
};

TEST_F(PredictionTest, ReplacingSnapshotRetiresOldCellAndNeverErasesWall)
{
  auto master = parent->getCostmap();
  master->setCost(10, 10, nav2_costmap_2d::LETHAL_OBSTACLE);
  send(.55, .55);
  bounds(); merge();
  EXPECT_EQ(master->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  send(.75, .55);
  bounds();
  EXPECT_LE(min_x, static_cast<double>(.55f));
  EXPECT_GE(max_x, .75);
  EXPECT_EQ(layer->getCost(5, 5), nav2_costmap_2d::NO_INFORMATION);
  // The layered map resets its dirty region before composing every layer.
  master->setCost(5, 5, nav2_costmap_2d::FREE_SPACE);
  merge();
  EXPECT_EQ(master->getCost(5, 5), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(master->getCost(7, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(master->getCost(10, 10), nav2_costmap_2d::LETHAL_OBSTACLE);
  send(0, 0, true);
  bounds();
  EXPECT_EQ(layer->getCost(7, 5), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_LE(min_x, .75);
}

TEST_F(PredictionTest, WrongFrameCannotMarkAndLostPublisherForecastExpires)
{
  send(.55, .55, false, "odom");
  bounds();
  EXPECT_EQ(layer->getCost(5, 5), nav2_costmap_2d::NO_INFORMATION);
  send(.55, .55);
  bounds();
  ASSERT_EQ(layer->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  std::this_thread::sleep_for(std::chrono::milliseconds(550));
  bounds();
  EXPECT_EQ(layer->getCost(5, 5), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_LE(min_x, static_cast<double>(.55f));
}

TEST_F(PredictionTest, FutureRiskIsNotPhysicalLethalAndCannotEraseActualWall)
{
  auto master = parent->getCostmap();
  master->setCost(5, 5, nav2_costmap_2d::LETHAL_OBSTACLE);
  send(.55, .55, false, "map", 200);
  bounds(); merge();
  EXPECT_EQ(layer->getCost(5, 5), 200);
  EXPECT_EQ(master->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  send(.75, .55, false, "map", 200);
  bounds(); merge();
  EXPECT_EQ(master->getCost(7, 5), 200);
  EXPECT_LT(master->getCost(7, 5), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

TEST_F(PredictionTest, HardPredictionCannotTrapRobotCurrentCell)
{
  send(.55, .55);
  min_x = min_y = std::numeric_limits<double>::infinity();
  max_x = max_y = -std::numeric_limits<double>::infinity();
  layer->updateBounds(.55, .55, 0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_EQ(layer->getCost(5, 5), nav2_costmap_2d::NO_INFORMATION);
  // The same fresh snapshot is restored as soon as the robot has left it.
  bounds();
  EXPECT_EQ(layer->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  // Soft future risk is intentionally not carved out.
  send(.75, .55, false, "map", 200);
  min_x = min_y = std::numeric_limits<double>::infinity();
  max_x = max_y = -std::numeric_limits<double>::infinity();
  layer->updateBounds(.75, .55, 0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_EQ(layer->getCost(7, 5), 200);
}

TEST_F(PredictionTest, LocalPredictionRequiresStampedTFAndFollowsRollingOrigin)
{
  layer.reset();
  parent = std::make_unique<nav2_costmap_2d::LayeredCostmap>("odom", true, false);
  parent->resizeMap(20, 20, .1, 0, 0);
  layer = std::make_unique<bot_navigation::PredictionLayer>();
  layer->initialize(parent.get(), "prediction", buffer.get(), lifecycle, group);
  send(.55, .55);
  bounds();
  EXPECT_EQ(layer->getCost(5, 5), nav2_costmap_2d::NO_INFORMATION);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "map";
  transform.header.stamp = lifecycle->now();
  transform.transform.translation.x = .2;
  transform.transform.translation.y = .3;
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(buffer->setTransform(transform, "unit", true));
  send(.55, .55);
  bounds();
  EXPECT_EQ(layer->getCost(7, 8), nav2_costmap_2d::LETHAL_OBSTACLE);
  parent->getCostmap()->updateOrigin(.5, 0);
  bounds();
  EXPECT_DOUBLE_EQ(layer->getOriginX(), .5);
  EXPECT_EQ(layer->getCost(2, 8), nav2_costmap_2d::LETHAL_OBSTACLE);
  send(0, 0, true);
  bounds();
  EXPECT_EQ(layer->getCost(2, 8), nav2_costmap_2d::NO_INFORMATION);
}
