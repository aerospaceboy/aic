// Copyright 2026 aerospaceboy
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace bot_navigation
{
struct PatrolPoint {double x, y;};
struct PatrolBlocker {PatrolPoint point; double clearance;};
struct PatrolStep {double phase; int direction; bool reversed; bool blocked;};

inline PatrolPoint patrolPoint(PatrolPoint a, PatrolPoint b, double phase)
{
  const double length=std::hypot(b.x-a.x,b.y-a.y);
  double distance=std::fmod(phase,2*length);
  if(distance<0)distance+=2*length;
  const double fraction=std::min(distance,2*length-distance)/length;
  return {a.x+(b.x-a.x)*fraction,a.y+(b.y-a.y)*fraction};
}

inline bool patrolSegmentClear(PatrolPoint a,PatrolPoint b,
  const std::vector<PatrolBlocker> & blockers)
{
  for(const auto & obstacle:blockers) {
    const double dx=b.x-a.x,dy=b.y-a.y, squared=dx*dx+dy*dy;
    const double t=squared>1.e-12 ? std::max(0.,std::min(1.,
      ((obstacle.point.x-a.x)*dx+(obstacle.point.y-a.y)*dy)/squared)):0.;
    const double start=std::hypot(a.x-obstacle.point.x,a.y-obstacle.point.y);
    const double finish=std::hypot(b.x-obstacle.point.x,b.y-obstacle.point.y);
    const double nearest=std::hypot(a.x+t*dx-obstacle.point.x,
      a.y+t*dy-obstacle.point.y);
    // If the robot has already entered our margin, permit only monotonic
    // separation. Never require the stopped robot to release a hysteresis.
    if(start<obstacle.clearance) {
      if(nearest+1.e-8<start || finish<=start+1.e-8)return false;
    } else if(nearest<obstacle.clearance)return false;
  }
  return true;
}

inline PatrolStep guardedPatrolStep(PatrolPoint a,PatrolPoint b,double phase,
  int direction,double distance,const std::vector<PatrolBlocker> & blockers)
{
  const auto current=patrolPoint(a,b,phase);
  for(int sign:{direction,-direction}) {
    const double next=phase+sign*distance;
    const double length=std::hypot(b.x-a.x,b.y-a.y);
    const double boundary=(sign>0 ? std::floor(phase/length)+1 :
      std::ceil(phase/length)-1)*length;
    bool clear;
    if(sign*(next-boundary)>=0) {
      const auto turning=patrolPoint(a,b,boundary);
      clear=patrolSegmentClear(current,turning,blockers) &&
        patrolSegmentClear(turning,patrolPoint(a,b,next),blockers);
    } else clear=patrolSegmentClear(current,patrolPoint(a,b,next),blockers);
    if(clear)
      return {next,sign,sign!=direction,false};
  }
  // A physically blocked rail has no collision-free motion. This is distinct
  // from the old unconditional 0.70 m proximity freeze, and is reported.
  return {phase,direction,false,true};
}
}  // namespace bot_navigation
