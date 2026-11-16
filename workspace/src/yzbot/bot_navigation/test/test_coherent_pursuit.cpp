// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include "gtest/gtest.h"
#include "nav2_core/controller.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2_ros/buffer.h"

class CoherentPursuitTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0,nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override {
    node=std::make_shared<nav2_util::LifecycleNode>("coherent_native","",
      rclcpp::NodeOptions().parameter_overrides({
        rclcpp::Parameter("controller_frequency",20.),
        rclcpp::Parameter("pursuit.desired_linear_vel",1.78),
        rclcpp::Parameter("pursuit.max_robot_pose_search_dist",1.5),
        rclcpp::Parameter("pursuit.use_collision_detection",true),
        rclcpp::Parameter("pursuit.max_angular_accel",5.),
        rclcpp::Parameter("pursuit.rotate_to_heading_angular_vel",1.8),
        rclcpp::Parameter("pursuit.heading_recovery_leave",.45),
        rclcpp::Parameter("pursuit.use_velocity_scaled_lookahead_dist",true),
        rclcpp::Parameter("pursuit.min_lookahead_dist",.6),
        rclcpp::Parameter("pursuit.max_lookahead_dist",1.2),
        rclcpp::Parameter("pursuit.lookahead_time",.8),
        rclcpp::Parameter("checker.xy_goal_tolerance",.15),
        rclcpp::Parameter("checker.yaw_goal_tolerance",6.28)}));
    costmap=std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      rclcpp::NodeOptions().arguments({"--ros-args","-r","__node:=coherent_native_map"}));
    costmap->set_parameters({
      rclcpp::Parameter("plugins",std::vector<std::string>{}),
      rclcpp::Parameter("global_frame","odom"),
      rclcpp::Parameter("robot_base_frame","base_footprint"),
      rclcpp::Parameter("robot_radius",.28),
      rclcpp::Parameter("footprint_padding",0.)});
    costmap->configure();
    costmap->getCostmap()->resizeMap(400,400,.05,-10.,-10.);
    costmap->getCostmap()->resetMap(0,0,400,400);
    buffer=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    loader=std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
      "nav2_core","nav2_core::Controller");
    controller=loader->createSharedInstance("bot_navigation::CoherentPursuitController");
    controller->configure(node,"pursuit",buffer,costmap);controller->activate();
    checker=std::make_unique<nav2_controller::SimpleGoalChecker>();
    checker->initialize(node,"checker",costmap);
  }
  void TearDown() override {
    controller->deactivate();controller->cleanup();controller.reset();loader.reset();
    costmap->cleanup();costmap.reset();node.reset();
  }
  geometry_msgs::msg::PoseStamped pose(double x,double y,double yaw) {
    geometry_msgs::msg::PoseStamped p;p.header.frame_id="odom";p.header.stamp=node->now();
    p.pose.position.x=x;p.pose.position.y=y;
    p.pose.orientation.z=std::sin(yaw/2);p.pose.orientation.w=std::cos(yaw/2);
    geometry_msgs::msg::TransformStamped t;t.header=p.header;t.child_frame_id="base_footprint";
    t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation=p.pose.orientation;
    EXPECT_TRUE(buffer->setTransform(t,"native",true));return p;
  }
  nav_msgs::msg::Path path(bool curve=false) {
    nav_msgs::msg::Path p;p.header.frame_id="odom";
    for(int i=0;i<=100;i++) {
      geometry_msgs::msg::PoseStamped q;q.header=p.header;q.pose.position.x=i*.05;
      q.pose.position.y=curve?.45*std::sin(q.pose.position.x*2*std::acos(-1.)/5):0.;
      q.pose.orientation.w=1.;p.poses.push_back(q);
    }
    return p;
  }
  std::shared_ptr<nav2_util::LifecycleNode> node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::shared_ptr<tf2_ros::Buffer> buffer;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader;
  nav2_core::Controller::Ptr controller;
  std::unique_ptr<nav2_controller::SimpleGoalChecker> checker;
};

TEST_F(CoherentPursuitTest, BehindTargetDoesNotProduceForwardMotion) {
  controller->setPlan(path());geometry_msgs::msg::Twist measured;
  auto command=controller->computeVelocityCommands(pose(0.,0.,std::acos(-1.)),measured,checker.get());
  EXPECT_DOUBLE_EQ(command.twist.linear.x,0.);EXPECT_NE(command.twist.angular.z,0.);
}

TEST_F(CoherentPursuitTest, FullCarriedFootprintStillRejectsRotationalCollision) {
  std::vector<geometry_msgs::msg::Point> footprint;
  for(const auto & xy:std::vector<std::pair<double,double>>{{-.28,-.28},{.28,-.28},
    {.46,-.12},{.46,.12},{.28,.28},{-.28,.28}}) {
    geometry_msgs::msg::Point p;p.x=xy.first;p.y=xy.second;footprint.push_back(p);
  }
  costmap->setRobotFootprint(footprint);unsigned int mx,my;
  ASSERT_TRUE(costmap->getCostmap()->worldToMap(-.32,-.32,mx,my));
  costmap->getCostmap()->setCost(mx,my,nav2_costmap_2d::LETHAL_OBSTACLE);
  controller->setPlan(path());geometry_msgs::msg::Twist measured;measured.angular.z=1.4;
  EXPECT_THROW(controller->computeVelocityCommands(
    pose(0.,0.,std::acos(-1.)-1.e-6),measured,checker.get()),std::exception);
}

TEST_F(CoherentPursuitTest, BoundedYawAccelerationTracksSCurveWithoutAFullLoop) {
  auto plan=path(true);controller->setPlan(plan);double x=0,y=0,yaw=std::acos(-1.);
  geometry_msgs::msg::Twist measured;double max_error=0.;int steps=0;
  for(;steps<600 && std::hypot(x-5.,y)>.16;steps++) {
    auto cmd=controller->computeVelocityCommands(pose(x,y,yaw),measured,checker.get()).twist;
    measured.linear.x+=std::clamp(cmd.linear.x-measured.linear.x,-4.5*.05,5.*.05);
    measured.angular.z+=std::clamp(cmd.angular.z-measured.angular.z,-4.*.05,4.*.05);
    x+=measured.linear.x*std::cos(yaw)*.05;y+=measured.linear.x*std::sin(yaw)*.05;
    yaw=std::remainder(yaw+measured.angular.z*.05,2*std::acos(-1.));
    double error=1.e6;
    for(const auto & p:plan.poses)error=std::min(error,
      std::hypot(x-p.pose.position.x,y-p.pose.position.y));
    max_error=std::max(max_error,error);
  }
  EXPECT_LT(steps*.05,15.);EXPECT_LT(std::hypot(x-5.,y),.16);EXPECT_LT(max_error,.35);
}
