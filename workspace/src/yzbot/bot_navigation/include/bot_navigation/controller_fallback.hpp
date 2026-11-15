// Copyright 2026 aerospaceboy
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <exception>
#include <string>
#include <cmath>
#include <algorithm>
#include <geometry_msgs/msg/twist.hpp>

namespace bot_navigation
{
inline void capTranslation(geometry_msgs::msg::Twist & command, double limit)
{
  const auto speed = std::hypot(command.linear.x, command.linear.y);
  if (limit > 0. && speed > limit) {
    command.linear.x *= limit / speed;
    command.linear.y *= limit / speed;
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
