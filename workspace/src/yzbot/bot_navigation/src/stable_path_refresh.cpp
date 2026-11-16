// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#include "bot_navigation/stable_path_refresh.hpp"
#include <algorithm>
#include <limits>
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bot_navigation
{
namespace
{
bool usable(const nav_msgs::msg::Path & path) {
  if (path.header.frame_id.empty() || path.poses.empty()) {return false;}
  for (const auto & p : path.poses) {
    if (!std::isfinite(p.pose.position.x) || !std::isfinite(p.pose.position.y) ||
      !std::isfinite(p.pose.position.z)) {return false;}
  }
  return true;
}
std::string key(const nav_msgs::msg::Path & path) {
  const auto & a = path.poses.front().pose.position;
  const auto & b = path.poses.back().pose.position;
  return path.header.frame_id + ":" + std::to_string(path.header.stamp.sec) + ":" +
    std::to_string(path.header.stamp.nanosec) + ":" + std::to_string(a.x) + ":" +
    std::to_string(a.y) + ":" + std::to_string(b.x) + ":" + std::to_string(b.y);
}
}
std::optional<RemainingGeometry> remainingGeometry(
  const nav_msgs::msg::Path & path, double x, double y)
{
  if (!usable(path) || !std::isfinite(x) || !std::isfinite(y)) {return std::nullopt;}
  if (path.poses.size()==1) {
    const auto & p=path.poses.front().pose.position;
    return RemainingGeometry{std::hypot(p.x-x,p.y-y),std::atan2(p.y-y,p.x-x)};
  }
  double best = std::numeric_limits<double>::max(), px = 0., py = 0.;
  size_t start = 1;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i-1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const double dx = b.x-a.x, dy = b.y-a.y, d2 = dx*dx+dy*dy;
    const double f = d2 > 1.e-12 ? std::clamp(((x-a.x)*dx+(y-a.y)*dy)/d2,0.,1.) : 0.;
    const double qx = a.x+f*dx, qy = a.y+f*dy, distance = std::hypot(qx-x,qy-y);
    if (distance < best) {best=distance; px=qx; py=qy; start=i;}
  }
  RemainingGeometry result; result.length=best;
  double travelled=0., hx=px, hy=py; bool heading_set=false;
  for (size_t i=start; i<path.poses.size(); ++i) {
    const auto & b=path.poses[i].pose.position;
    const double length=std::hypot(b.x-px,b.y-py);
    if (!heading_set && travelled+length >= .8 && length > 1.e-12) {
      const double f=(.8-travelled)/length; hx=px+f*(b.x-px); hy=py+f*(b.y-py);
      heading_set=true;
    }
    travelled+=length; result.length+=length; px=b.x; py=b.y;
    if (!heading_set) {hx=px; hy=py;}
  }
  result.heading=std::atan2(hy-y,hx-x);
  return result;
}
bool worthwhilePathChange(const RemainingGeometry & old_path,
  const RemainingGeometry & candidate, double yaw, double min_gain,
  double min_fraction, double mean_speed, double turn_rate)
{
  if (!std::isfinite(old_path.length+old_path.heading+candidate.length+candidate.heading+
      yaw+min_gain+min_fraction+mean_speed+turn_rate) || mean_speed <= 0. || turn_rate <= 0. ||
      min_gain < 0. || min_fraction <= 0. || min_fraction > 1.) {return false;}
  const double gain=old_path.length-candidate.length;
  const double pi2=2*std::acos(-1.);
  const double extra_turn=std::max(0., std::abs(std::remainder(candidate.heading-yaw,pi2))-
    std::abs(std::remainder(old_path.heading-yaw,pi2)));
  return gain >= std::max(min_gain,old_path.length*min_fraction) &&
    gain/mean_speed-extra_turn/turn_rate >= 1.;
}
StablePathRefresh::StablePathRefresh(const std::string & name, const BT::NodeConfiguration & config)
: StablePathRefresh(name,config,[]() {return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();}) {}
StablePathRefresh::StablePathRefresh(const std::string & name,
  const BT::NodeConfiguration & config, Clock clock)
: BT::ControlNode(name,config), clock_(std::move(clock)) {
  node_=config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_=config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}
BT::PortsList StablePathRefresh::providedPorts() {
  return {BT::BidirectionalPort<nav_msgs::msg::Path>("path"),
    BT::InputPort<nav_msgs::msg::Path>("candidate"),
    BT::InputPort<double>("interval",3.,"Optional refresh interval"),
    BT::InputPort<double>("min_gain",1.5,"Minimum shortening in metres"),
    BT::InputPort<double>("min_fraction",.12,"Minimum relative shortening"),
    BT::InputPort<std::string>("robot_base_frame","base_footprint","Robot frame")};
}
void StablePathRefresh::halt() {planning_=false; BT::ControlNode::halt();}
BT::NodeStatus StablePathRefresh::tick() {
  nav_msgs::msg::Path old_path,candidate;
  double interval=0.,gain=0.,fraction=0.; std::string frame;
  if (children_nodes_.size()!=2 || !getInput("path",old_path) || !usable(old_path) ||
    !getInput("interval",interval) || !getInput("min_gain",gain) ||
    !getInput("min_fraction",fraction) || !getInput("robot_base_frame",frame) ||
    !std::isfinite(interval+gain+fraction) || interval<1. || interval>10. ||
    gain<.5 || fraction<=0. || fraction>.5) {return BT::NodeStatus::FAILURE;}
  const double now=clock_();
  if (!std::isfinite(now) || (last_tick_ && now<*last_tick_)) {halt(); return BT::NodeStatus::FAILURE;}
  last_tick_=now;
  if (path_key_!=key(old_path)) {
    if (planning_) {resetChildren(); planning_=false;}
    path_key_=key(old_path); last_refresh_=now;
  }
  if (!planning_) {
    const auto valid=children_nodes_[0]->executeTick();
    if (valid==BT::NodeStatus::RUNNING) {return valid;}
    haltChild(0); mandatory_=valid!=BT::NodeStatus::SUCCESS;
    if (!mandatory_) {
      if (!last_refresh_ || now-*last_refresh_<interval) {return BT::NodeStatus::SUCCESS;}
      try {
        const auto pose=tf_->lookupTransform(old_path.header.frame_id,frame,tf2::TimePointZero);
        const auto geometry=remainingGeometry(old_path,pose.transform.translation.x,pose.transform.translation.y);
        if (!geometry || geometry->length<8.) {last_refresh_=now; return BT::NodeStatus::SUCCESS;}
      } catch (const tf2::TransformException &) {last_refresh_=now; return BT::NodeStatus::SUCCESS;}
    }
    planning_=true;
  }
  const auto status=children_nodes_[1]->executeTick();
  if (status==BT::NodeStatus::RUNNING) {return status;}
  planning_=false; last_refresh_=now; haltChild(1);
  if (status!=BT::NodeStatus::SUCCESS || !getInput("candidate",candidate) || !usable(candidate))
    return mandatory_ ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  if (mandatory_) {setOutput("path",candidate); path_key_=key(candidate); return BT::NodeStatus::SUCCESS;}
  const auto & a=old_path.poses.back().pose.position;
  const auto & b=candidate.poses.back().pose.position;
  if (old_path.header.frame_id!=candidate.header.frame_id || std::hypot(a.x-b.x,a.y-b.y)>.1)
    return BT::NodeStatus::SUCCESS;
  try {
    const auto pose=tf_->lookupTransform(old_path.header.frame_id,frame,tf2::TimePointZero);
    const double x=pose.transform.translation.x,y=pose.transform.translation.y;
    const auto old_geometry=remainingGeometry(old_path,x,y);
    const auto next_geometry=remainingGeometry(candidate,x,y);
    if (old_geometry && next_geometry && worthwhilePathChange(
        *old_geometry,*next_geometry,tf2::getYaw(pose.transform.rotation),gain,fraction)) {
      setOutput("path",candidate); path_key_=key(candidate);
      RCLCPP_INFO(node_->get_logger(),"path_refinement accepted old_m=%.2f new_m=%.2f gain_m=%.2f",
        old_geometry->length,next_geometry->length,old_geometry->length-next_geometry->length);
    }
  } catch (const tf2::TransformException &) {}
  return BT::NodeStatus::SUCCESS;
}
}  // namespace bot_navigation
