// Copyright 2026 aerospaceboy
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>

namespace bot_navigation
{
inline bool openSpaceAllowed(double clearance, double age, bool active)
{
  // Distance is 1-Lipschitz. Account for all travel since the stamped pose,
  // rather than treating a fresh status envelope as a fresh odometry sample.
  return std::isfinite(clearance+age) && age>=-.05 && age<.35 &&
    clearance-1.78*std::max(0.,age)>(active?.80:1.0);
}
// Finite prediction cost. Current occupancy and final collision guards remain
// hard; uncertain long-range forecasts must not invalidate every escape.
inline double timedSeparationRisk(double separation, double proximity_weight,
  double collision_weight)
{
  const auto near = proximity_weight * std::max(0., .35 - separation);
  return near + (separation < 0. ? collision_weight *
    (1. + std::min(1., -separation / .20)) : 0.);
}
}  // namespace bot_navigation
