// Copyright 2026 aerospaceboy
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>

namespace bot_navigation
{
inline bool continuousPathEligible(double remaining, double path_distance, bool active)
{
  return std::isfinite(remaining + path_distance) && path_distance >= 0. &&
    remaining > (active ? .8 : 1.2) && path_distance < (active ? .60 : .40);
}

inline bool openSpaceAllowed(double clearance, double age, bool active)
{
  // Distance is 1-Lipschitz. Account for all travel since the stamped pose,
  // rather than treating a fresh status envelope as a fresh odometry sample.
  // Full MPPI rollouts now independently enforce 0.48 m static clearance
  // plus cell quantization. This arbitration reserve need not require a
  // one-metre open plaza and exclude otherwise feasible central passages.
  return std::isfinite(clearance+age) && age>=-.05 && age<.35 &&
    clearance-1.78*std::max(0.,age)>(active?.60:.70);
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
