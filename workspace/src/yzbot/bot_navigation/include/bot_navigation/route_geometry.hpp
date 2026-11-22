// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include "nav_msgs/msg/path.hpp"

namespace bot_navigation
{
struct RemainingGeometry {double length{0.}, heading{0.};};
// The BT and controller must identify the SAME unpruned source path. A goal
// alone is insufficient: mandatory replans can retain that goal while changing
// every preceding segment. Never compare progress from another controller plan.
inline std::string routeKey(const nav_msgs::msg::Path & path)
{
  uint64_t hash=14695981039346656037ULL;
  auto mix=[&](uint64_t value) {for(int i=0;i<8;++i) {hash^=value&255;hash*=1099511628211ULL;value>>=8;}};
  for(unsigned char ch:path.header.frame_id) {hash^=ch;hash*=1099511628211ULL;}
  mix(static_cast<uint64_t>(path.header.stamp.sec));mix(path.header.stamp.nanosec);
  mix(path.poses.size());
  for(const auto & p:path.poses) {
    for(double value:{p.pose.position.x,p.pose.position.y,p.pose.position.z,
      p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w}) {
      uint64_t bits;std::memcpy(&bits,&value,sizeof(bits));mix(bits);
    }
  }
  return std::to_string(hash);
}
// Use ONLY the controller's already-pruned suffix, in its original frame.
// Searching an entire looping path for the nearest segment can jump across a
// wall to a later segment and erase a real detour from the cost estimate.
inline std::optional<RemainingGeometry> suffixGeometry(
  const nav_msgs::msg::Path & suffix, double x, double y)
{
  if(suffix.poses.empty() || !std::isfinite(x) || !std::isfinite(y))return std::nullopt;
  RemainingGeometry result;double px=x,py=y,travelled=0.,hx=x,hy=y;bool heading=false;
  for(const auto & pose:suffix.poses) {
    const double nx=pose.pose.position.x,ny=pose.pose.position.y;
    if(!std::isfinite(nx) || !std::isfinite(ny))return std::nullopt;
    const double length=std::hypot(nx-px,ny-py);
    if(!heading && travelled+length>=.8 && length>1.e-12) {
      const double f=(.8-travelled)/length;hx=px+f*(nx-px);hy=py+f*(ny-py);heading=true;
    }
    travelled+=length;px=nx;py=ny;
    if(!heading) {hx=px;hy=py;}
  }
  result.length=travelled;result.heading=std::atan2(hy-y,hx-x);return result;
}
}  // namespace bot_navigation
