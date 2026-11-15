// Copyright 2026 aerospaceboy
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>

namespace bot_navigation
{
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
