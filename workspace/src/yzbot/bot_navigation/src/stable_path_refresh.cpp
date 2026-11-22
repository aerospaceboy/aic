// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#include "bot_navigation/stable_path_refresh.hpp"
#include <algorithm>
#include <limits>
#include <nlohmann/json.hpp>
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
  double searched=0.;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & a = path.poses[i-1].pose.position;
    const auto & b = path.poses[i].pose.position;
    const double dx = b.x-a.x, dy = b.y-a.y, d2 = dx*dx+dy*dy;
    if (i>1 && searched>1.5) {break;}
    searched+=std::sqrt(d2);
    const double f = d2 > 1.e-12 ? std::clamp(((x-a.x)*dx+(y-a.y)*dy)/d2,0.,1.) : 0.;
    const double qx = a.x+f*dx, qy = a.y+f*dy, distance = std::hypot(qx-x,qy-y);
    if (distance < best) {best=distance; px=qx; py=qy; start=i;}
  }
  if (best>.6) {return std::nullopt;}
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
  group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
  rclcpp::SubscriptionOptions options;options.callback_group=group_;
  subscription_=node_->create_subscription<std_msgs::msg::String>(
    "/competition/pursuit/progress",rclcpp::QoS(1),
    [this](std_msgs::msg::String::ConstSharedPtr msg) {receiveProgress(*msg);},options);
  executor_.add_callback_group(group_,node_->get_node_base_interface());
}
void StablePathRefresh::receiveProgress(const std_msgs::msg::String & message) {
  if(message.data.size()>2048)return;
  try {
    const auto j=nlohmann::json::parse(message.data);
    Progress p{j.at("path_key").get<std::string>(),
      {j.at("remaining_m").get<double>(),j.at("heading").get<double>()},
      j.at("x").get<double>(),j.at("y").get<double>(),j.at("stamp").get<double>(),
      clock_(),j.at("tracking").get<bool>()};
    if(!std::isfinite(p.geometry.length+p.geometry.heading+p.x+p.y+p.stamp+p.received) ||
      p.geometry.length<0. || p.key.size()>32)return;
    progress_=p;
  } catch(const nlohmann::json::exception &) {}
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
  executor_.spin_some(std::chrono::milliseconds(2));
  nav_msgs::msg::Path old_path,candidate;
  double interval=0.,gain=0.,fraction=0.;std::string frame;
  if(children_nodes_.size()!=3 || !getInput("path",old_path) || !usable(old_path) ||
    !getInput("interval",interval) || !getInput("min_gain",gain) ||
    !getInput("min_fraction",fraction) || !getInput("robot_base_frame",frame) ||
    !std::isfinite(interval+gain+fraction) || interval<1. || interval>10. ||
    gain<.5 || fraction<=0. || fraction>.5)return BT::NodeStatus::FAILURE;
  const double now=clock_();
  if(!std::isfinite(now) || (last_tick_ && now<*last_tick_)) {halt();return BT::NodeStatus::FAILURE;}
  last_tick_=now;
  const auto key=routeKey(old_path);
  if(path_key_!=key) {
    if(planning_) {resetChildren();planning_=false;}
    path_key_=key;last_refresh_=now;
  }
  auto pose=[&]() {return tf_->lookupTransform(old_path.header.frame_id,frame,tf2::TimePointZero);};
  auto fresh=[&](double x,double y) {
    if(!progress_ || progress_->key!=key || !progress_->tracking)return false;
    const double age=node_->now().seconds()-progress_->stamp;
    return age>=-.05 && age<=.5 && now-progress_->received>=0. && now-progress_->received<=.5 &&
      std::hypot(progress_->x-x,progress_->y-y)<=.5;
  };
  if(!planning_) {
    const auto valid=children_nodes_[0]->executeTick();
    if(valid==BT::NodeStatus::RUNNING)return valid;
    haltChild(0);mandatory_=valid!=BT::NodeStatus::SUCCESS;
    if(!mandatory_) {
      if(!last_refresh_ || now-*last_refresh_<interval)return BT::NodeStatus::SUCCESS;
      last_refresh_=now;
      try {
        const auto p=pose();
        if(!fresh(p.transform.translation.x,p.transform.translation.y) || progress_->geometry.length<6.) {
          RCLCPP_INFO(node_->get_logger(),"path_refinement skipped reason=no_fresh_tracking_suffix_or_short_route");
          return BT::NodeStatus::SUCCESS;
        }
      } catch(const tf2::TransformException &) {return BT::NodeStatus::SUCCESS;}
    }
    try {const auto p=pose();request_x_=p.transform.translation.x;request_y_=p.transform.translation.y;}
    catch(const tf2::TransformException &) {if(!mandatory_)return BT::NodeStatus::SUCCESS;}
    requested_at_=now;planning_=true;
  }
  if(!mandatory_ && now-requested_at_>1.) {
    haltChild(1);planning_=false;last_refresh_=now;
    RCLCPP_INFO(node_->get_logger(),"path_refinement rejected reason=planning_timeout");
    return BT::NodeStatus::SUCCESS;
  }
  const auto status=children_nodes_[1]->executeTick();
  if(status==BT::NodeStatus::RUNNING)return status;
  planning_=false;last_refresh_=now;haltChild(1);
  if(status!=BT::NodeStatus::SUCCESS || !getInput("candidate",candidate) || !usable(candidate)) {
    RCLCPP_INFO(node_->get_logger(),"path_refinement rejected reason=planner_failed mandatory=%s",mandatory_?"true":"false");
    return mandatory_?BT::NodeStatus::FAILURE:BT::NodeStatus::SUCCESS;
  }
  if(mandatory_) {setOutput("path",candidate);path_key_=routeKey(candidate);return BT::NodeStatus::SUCCESS;}
  const auto & a=old_path.poses.back().pose.position;
  const auto & b=candidate.poses.back().pose.position;
  if(old_path.header.frame_id!=candidate.header.frame_id || std::hypot(a.x-b.x,a.y-b.y)>.1)
    return BT::NodeStatus::SUCCESS;
  try {
    const auto p=pose();const double x=p.transform.translation.x,y=p.transform.translation.y;
    const double pose_age=node_->now().seconds()-rclcpp::Time(p.header.stamp,node_->get_clock()->get_clock_type()).seconds();
    if(!fresh(x,y) || now-requested_at_>1. || std::hypot(x-request_x_,y-request_y_)>.5 ||
      pose_age<-.05 || pose_age>.5) {
      RCLCPP_INFO(node_->get_logger(),"path_refinement rejected reason=stale_progress_or_candidate");
      return BT::NodeStatus::SUCCESS;
    }
    const auto next=remainingGeometry(candidate,x,y);
    if(!next || !worthwhilePathChange(progress_->geometry,*next,tf2::getYaw(p.transform.rotation),gain,fraction)) {
      RCLCPP_INFO(node_->get_logger(),"path_refinement rejected reason=insufficient_net_gain old_m=%.2f new_m=%.2f",
        progress_->geometry.length,next?next->length:-1.);
      return BT::NodeStatus::SUCCESS;
    }
    const auto valid=children_nodes_[2]->executeTick();haltChild(2);
    if(valid!=BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(),"path_refinement rejected reason=candidate_not_valid");
      return BT::NodeStatus::SUCCESS;
    }
    setOutput("path",candidate);path_key_=routeKey(candidate);
    RCLCPP_INFO(node_->get_logger(),"path_refinement accepted old_m=%.2f new_m=%.2f gain_m=%.2f",
      progress_->geometry.length,next->length,progress_->geometry.length-next->length);
  } catch(const tf2::TransformException &) {}
  return BT::NodeStatus::SUCCESS;
}
}  // namespace bot_navigation
