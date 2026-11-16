// Copyright 2026 YZBot contributors
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bot_navigation
{
class HeadingRecovery
{
public:
  HeadingRecovery(double enter = 1.2, double leave = .35)
  : enter_(enter), leave_(leave) {
    if (!std::isfinite(enter + leave) || leave <= 0 || leave >= enter || enter >= 3.14)
      throw std::invalid_argument("Invalid pursuit heading hysteresis");
  }
  bool update(double bearing) {
    if (!std::isfinite(bearing))throw std::invalid_argument("Nonfinite path bearing");
    const double error = std::abs(std::remainder(bearing, 2 * std::acos(-1.)));
    active_ = active_ ? error > leave_ : error > enter_;
    return active_;
  }
  void reset() {active_ = false;}
private:
  double enter_, leave_;bool active_{false};
};

// Uniformly time-scale a checked pursuit arc while the yaw actuator catches
// up. Incompatible sign / excessive measured yaw requires braking, never an
// independent angular clamp that changes the arc's curvature.
inline double pursuitTimeScale(double requested_yaw, double measured_yaw,
  double acceleration, double dt, double noise = .03)
{
  if (!std::isfinite(requested_yaw + measured_yaw + acceleration + dt + noise) ||
      acceleration <= 0 || dt <= 0 || noise < 0)
    throw std::invalid_argument("Invalid pursuit dynamics");
  const double step = acceleration * dt;
  if (std::abs(requested_yaw - measured_yaw) <= step + noise)return 1.;
  if (std::abs(requested_yaw) < 1.e-6)return 0.;
  const double next = std::clamp(requested_yaw, measured_yaw-step, measured_yaw+step);
  if (next * requested_yaw <= 0 || std::abs(next) > std::abs(requested_yaw))return 0.;
  return std::clamp(std::abs(next / requested_yaw), 0., 1.);
}
}  // namespace bot_navigation
