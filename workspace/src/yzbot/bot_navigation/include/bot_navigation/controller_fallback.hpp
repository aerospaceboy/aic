// Copyright 2026 aerospaceboy
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <exception>
#include <string>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <geometry_msgs/msg/twist.hpp>

namespace bot_navigation
{
inline void capTranslation(geometry_msgs::msg::Twist & command, double limit)
{
  const auto speed = std::hypot(command.linear.x, command.linear.y);
  if (limit > 0. && speed > limit) {
    command.linear.x *= limit / speed;
    command.linear.y *= limit / speed;
    command.angular.z *= limit / speed;
  }
}

// A long pursuit preview can cut a mapped corner although a shorter arc is
// feasible. Use a SECOND collision-checked computation only after braking;
// TF, stale-path and other errors must retain the ordinary recovery path.
template<class Command, class Primary, class Tight>
Command collisionPreview(Primary primary,Tight tight,bool enabled,
  double measured_speed,double maximum,bool & used,std::string & reason)
{
  used=false;
  try {return primary();}
  catch(const std::exception & error) {
    std::string text=error.what();
    std::transform(text.begin(),text.end(),text.begin(),
      [](unsigned char c){return std::tolower(c);});
    if(!enabled || !std::isfinite(measured_speed) || measured_speed>maximum+.05 ||
      text.find("collision")==std::string::npos)throw;
    Command command=tight();  // Its own unchanged collision checker must pass.
    used=true;reason=error.what();
    return command;
  }
}
// Fall back only to a command successfully computed from CURRENT observations.
// An invalid nominal command must never turn an optimizer failure into motion.
template<class Command, class Compute>
Command dynamicWithFallback(Compute compute, const Command & nominal,
  std::exception_ptr nominal_error, bool & dynamic, std::string & failure)
{
  try {return compute();}
  catch (const std::exception & error) {
    if (nominal_error) {throw;}
    dynamic = false;
    failure = error.what();
    return nominal;
  }
}
}  // namespace bot_navigation
