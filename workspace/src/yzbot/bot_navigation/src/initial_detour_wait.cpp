// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#include "bot_navigation/initial_detour_wait.hpp"
#include <algorithm>
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bot_navigation
{
InitialDetourWait::InitialDetourWait(const std::string & name,const BT::NodeConfiguration & config)
: InitialDetourWait(name,config,[]() {return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();}) {}
InitialDetourWait::InitialDetourWait(const std::string & name,
  const BT::NodeConfiguration & config,Clock clock)
: BT::ControlNode(name,config),clock_(std::move(clock)) {
  node_=config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_=config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}
BT::PortsList InitialDetourWait::providedPorts() {
  return {BT::BidirectionalPort<nav_msgs::msg::Path>("path"),
    BT::InputPort<nav_msgs::msg::Path>("candidate"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::InputPort<double>("max_wait",2.5,"Bounded initial detour opportunity"),
    BT::InputPort<std::string>("robot_base_frame","base_footprint","Robot frame")};
}
void InitialDetourWait::halt() {
  planning_=false;BT::ControlNode::halt();
  // A recovery for the SAME goal must not buy another waiting budget.
}
BT::NodeStatus InitialDetourWait::tick() {
  nav_msgs::msg::Path path,candidate;geometry_msgs::msg::PoseStamped goal;
  double budget=0.;std::string frame;
  if(children_nodes_.size()!=2 || !getInput("path",path) || path.poses.empty() ||
    !getInput("goal",goal) ||
    !getInput("max_wait",budget) || !std::isfinite(budget) || budget<0. || budget>3. ||
    !getInput("robot_base_frame",frame) || path.header.frame_id!=goal.header.frame_id)
    return BT::NodeStatus::FAILURE;
  const auto & endpoint=path.poses.back().pose.position;
  if(!std::isfinite(goal.pose.position.x+goal.pose.position.y) ||
    std::hypot(endpoint.x-goal.pose.position.x,endpoint.y-goal.pose.position.y)>.55) {
    halt();return BT::NodeStatus::FAILURE;
  }
  nav_msgs::msg::Path identity;identity.header.frame_id=goal.header.frame_id;identity.poses={goal};
  const auto key=routeKey(identity);const double now=clock_();
  if(!std::isfinite(now))return BT::NodeStatus::FAILURE;
  if(done_goal_==key)return BT::NodeStatus::SUCCESS;
  if(goal_key_!=key) {
    if(started_) {halt();started_.reset();last_attempt_.reset();goal_key_=key;return BT::NodeStatus::FAILURE;}
    goal_key_=key;
  }
  auto finish=[&]() {if(planning_)haltChild(0);planning_=false;done_goal_=key;started_.reset();last_attempt_.reset();
    return BT::NodeStatus::SUCCESS;};
  try {
    const auto pose=tf_->lookupTransform(path.header.frame_id,frame,tf2::TimePointZero);
    const double x=pose.transform.translation.x,y=pose.transform.translation.y;
    const double yaw=tf2::getYaw(pose.transform.rotation);
    const double age=node_->now().seconds()-rclcpp::Time(pose.header.stamp,node_->get_clock()->get_clock_type()).seconds();
    if(age<-.15 || age>.5)return finish();
    const auto geometry=remainingGeometry(path,x,y);
    const double direct=std::hypot(goal.pose.position.x-x,goal.pose.position.y-y);
    if(!geometry || !std::isfinite(direct+yaw) || budget==0.)return finish();
    if(!started_) {
      // A Euclidean lower bound only selects a SMALL, bounded opportunity;
      // it is never driven or treated as an obstacle-free reference path.
      if(geometry->length<20. || geometry->length<1.45*direct || geometry->length-direct<8.)return finish();
      started_=now;start_x_=x;start_y_=y;
      RCLCPP_INFO(node_->get_logger(),"initial_detour_window begin path_m=%.2f lower_bound_m=%.2f max_wait=%.2f",
        geometry->length,direct,budget);
    }
    if(now<*started_)return BT::NodeStatus::FAILURE;
    if(now-*started_>=budget || std::hypot(x-start_x_,y-start_y_)>.3) {
      RCLCPP_INFO(node_->get_logger(),"initial_detour_window complete reason=bounded_window_or_movement");
      return finish();
    }
    if(!planning_) {
      if(last_attempt_ && now-*last_attempt_<.4)return BT::NodeStatus::RUNNING;
      last_attempt_=now;planning_=true;
    }
    const auto status=children_nodes_[0]->executeTick();
    if(status==BT::NodeStatus::RUNNING)return status;
    planning_=false;haltChild(0);
    if(status!=BT::NodeStatus::SUCCESS || !getInput("candidate",candidate) || candidate.poses.empty() ||
      candidate.header.frame_id!=goal.header.frame_id)return BT::NodeStatus::RUNNING;
    const auto & end=candidate.poses.back().pose.position;
    if(std::hypot(end.x-goal.pose.position.x,end.y-goal.pose.position.y)>.55)
      return BT::NodeStatus::RUNNING;
    const auto next=remainingGeometry(candidate,x,y);
    if(!next || !worthwhilePathChange(*geometry,*next,yaw,1.5,.12))return BT::NodeStatus::RUNNING;
    const double pi2=2*std::acos(-1.);
    const double turn=std::max(0.,std::abs(std::remainder(next->heading-yaw,pi2))-
      std::abs(std::remainder(geometry->heading-yaw,pi2)));
    const double gain_seconds=(geometry->length-next->length)/1.3-turn/1.4-(now-*started_);
    if(gain_seconds<1.)return BT::NodeStatus::RUNNING;
    const auto valid=children_nodes_[1]->executeTick();haltChild(1);
    if(valid!=BT::NodeStatus::SUCCESS)return BT::NodeStatus::RUNNING;
    // Planner/service calls may consume the entire bounded opportunity.
    if(clock_()-*started_>=budget)return finish();
    setOutput("path",candidate);
    RCLCPP_INFO(node_->get_logger(),"initial_detour_window accepted old_m=%.2f new_m=%.2f wait=%.3f net_estimate_sec=%.2f",
      geometry->length,next->length,now-*started_,gain_seconds);
    return finish();
  } catch(const tf2::TransformException &) {return finish();}
}
}  // namespace bot_navigation
