// Copyright 2026 aerospaceboy
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <nav_msgs/msg/path.hpp>

namespace bot_navigation
{
// An inactive child has not pruned its private path as the nominal child drove.
// Preserve one segment behind the current projection and the exact final goal.
inline nav_msgs::msg::Path remainingPath(const nav_msgs::msg::Path & path, size_t nearest)
{
  auto result = path;
  if (result.poses.empty()) {return result;}
  const auto index = std::min(nearest, result.poses.size() - 1);
  const auto begin = index > 0 ? index - 1 : 0;
  result.poses.erase(result.poses.begin(), result.poses.begin() + begin);
  return result;
}
}  // namespace bot_navigation
