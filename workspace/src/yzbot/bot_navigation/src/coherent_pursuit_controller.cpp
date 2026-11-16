// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
// Derived from Nav2 Humble RegulatedPurePursuitController's command pipeline:
// https://github.com/ros-navigation/navigation2/tree/humble/nav2_regulated_pure_pursuit_controller
// Reuse its path transform/pruning, cost regulation and FULL footprint check.
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <nlohmann/json.hpp>
#include <nav2_core/exceptions.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/string.hpp>
#include "bot_navigation/pursuit_dynamics.hpp"

namespace bot_navigation
{
class CoherentPursuitController :
  public nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    RegulatedPurePursuitController::configure(parent, name, tf, costmap);
    auto node = parent.lock();name_ = name;
    nav2_util::declare_parameter_if_not_declared(node, name+".heading_recovery_enter",
      rclcpp::ParameterValue(1.2));
    nav2_util::declare_parameter_if_not_declared(node, name+".heading_recovery_leave",
      rclcpp::ParameterValue(.35));
    nav2_util::declare_parameter_if_not_declared(node, name+".tracking_angular_accel",
      rclcpp::ParameterValue(4.0));
    nav2_util::declare_parameter_if_not_declared(node, name+".curvature_accel_gate",
      rclcpp::ParameterValue(true));
    heading_ = HeadingRecovery(node->get_parameter(name+".heading_recovery_enter").as_double(),
      node->get_parameter(name+".heading_recovery_leave").as_double());
    angular_accel_ = node->get_parameter(name+".tracking_angular_accel").as_double();
    acceleration_gate_ = node->get_parameter(name+".curvature_accel_gate").as_bool();
    if (!std::isfinite(angular_accel_) || angular_accel_ <= 0 ||
      angular_accel_ > max_angular_accel_ || allow_reversing_ || !use_collision_detection_)
      throw std::invalid_argument("Coherent pursuit requires forward collision-checked dynamics");
    status_ = node->create_publisher<std_msgs::msg::String>("/competition/pursuit/state",10);
  }
  void activate() override {
    RegulatedPurePursuitController::activate();status_->on_activate();heading_.reset();
  }
  void deactivate() override {
    status_->on_deactivate();RegulatedPurePursuitController::deactivate();
  }
  void cleanup() override {status_.reset();RegulatedPurePursuitController::cleanup();}
  void setPlan(const nav_msgs::msg::Path & path) override {
    std::lock_guard<std::mutex> lock(mutex_);
    // Replanning the same goal must not rearm entry alignment each cycle.
    if (path.poses.empty() || global_plan_.poses.empty() ||
      path.header.frame_id != global_plan_.header.frame_id ||
      std::hypot(path.poses.back().pose.position.x-global_plan_.poses.back().pose.position.x,
                 path.poses.back().pose.position.y-global_plan_.poses.back().pose.position.y)>.10)
      heading_.reset();
    RegulatedPurePursuitController::setPlan(path);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & speed, nav2_core::GoalChecker * checker) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto costmap = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap->getMutex());
    geometry_msgs::msg::Pose pose_tolerance;geometry_msgs::msg::Twist velocity_tolerance;
    if (!checker || !checker->getTolerances(pose_tolerance,velocity_tolerance))
      throw nav2_core::PlannerException("Pursuit goal tolerance unavailable");
    goal_dist_tol_ = pose_tolerance.position.x;
    if (!std::isfinite(goal_dist_tol_) || goal_dist_tol_ <= 0 ||
      !std::isfinite(speed.linear.x) || !std::isfinite(speed.angular.z))
      throw nav2_core::PlannerException("Pursuit state or goal tolerance invalid");
    auto plan = transformGlobalPlan(pose);
    auto carrot = getLookAheadPoint(getLookAheadDistance(speed),plan);
    carrot_pub_->publish(createCarrotMsg(carrot));
    const double x=carrot.pose.position.x,y=carrot.pose.position.y;
    if (!std::isfinite(x) || !std::isfinite(y))
      throw nav2_core::PlannerException("Pursuit path contains nonfinite coordinates");
    const double distance=std::hypot(x,y),bearing=std::atan2(y,x);
    const double curvature=distance*distance>.001 ? 2*y/(distance*distance) : 0.;
    double v=desired_linear_vel_,w=0.,scale=1.;
    geometry_msgs::msg::PoseStamped robot_in_plan;
    if (!transformPose(global_plan_.header.frame_id,pose,robot_in_plan))
      throw nav2_core::PlannerException("Pursuit goal frame unavailable");
    const auto & end=global_plan_.poses.back().pose.position;
    // Final yaw belongs to the existing task/docking owner. A wide yaw goal
    // checker must not turn a transit endpoint into a new alignment action.
    const bool reached=std::hypot(end.x-robot_in_plan.pose.position.x,
      end.y-robot_in_plan.pose.position.y)<=goal_dist_tol_;
    const bool align=!reached && heading_.update(bearing);
    if (reached) {heading_.reset();v=0.;}
    else if (align) {rotateToHeading(v,w,bearing,speed);}
    else {
      double sign=1.;
      applyConstraints(curvature,speed,costAtPose(pose.pose.position.x,pose.pose.position.y),
        plan,v,sign);
      w=v*curvature;
      if (acceleration_gate_) {
        scale=pursuitTimeScale(w,speed.angular.z,angular_accel_,control_duration_);
        v*=scale;w*=scale;
      }
    }
    // Check the ACTUAL selected arc after time scaling / heading recovery.
    // A failed check propagates to the existing preview/bounded recovery.
    if (isCollisionImminent(pose,v,w,distance))
      throw nav2_core::PlannerException("RegulatedPurePursuitController detected collision ahead!");
    geometry_msgs::msg::TwistStamped command;command.header=pose.header;
    command.twist.linear.x=v;command.twist.angular.z=w;
    const double now=clock_->now().seconds();
    if (now-last_status_>=.19 || align!=last_align_) {
      nlohmann::json j={{"stamp",now},{"plugin",name_},{"mode",align?"heading_recovery":"tracking"},
        {"bearing",bearing},{"carrot_distance",distance},{"curvature_scale",scale},
        {"measured_yaw_rate",speed.angular.z},{"selected_command",{v,0.,w}}};
      j["pose_age"]=now-rclcpp::Time(pose.header.stamp,clock_->get_clock_type()).seconds();
      std_msgs::msg::String m;m.data=j.dump();status_->publish(m);
      last_status_=now;last_align_=align;
    }
    return command;
  }
private:
  HeadingRecovery heading_;
  double angular_accel_{4.},last_status_{-100.};bool acceleration_gate_{true},last_align_{false};
  std::string name_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_;
};
}  // namespace bot_navigation
PLUGINLIB_EXPORT_CLASS(bot_navigation::CoherentPursuitController,nav2_core::Controller)
