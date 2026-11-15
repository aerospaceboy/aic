// Copyright 2026 aerospaceboy
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

namespace bot_navigation
{
// Immutable static geometry for MPPI only. RPP keeps its original collision
// projection; adding this shell to their shared costmap would slow both.
class StaticClearanceGrid
{
public:
  void build(const std::vector<int8_t> & cells, unsigned width, unsigned height,
    double resolution, double ox, double oy, double yaw, double clearance)
  {
    if (!width || !height || cells.size()!=size_t(width)*height ||
      !std::isfinite(resolution+ox+oy+yaw+clearance) || resolution<=0. || clearance<.44)
      throw std::invalid_argument("Invalid static trajectory clearance grid");
    width_=width;height_=height;resolution_=resolution;ox_=ox;oy_=oy;
    cosine_=std::cos(yaw);sine_=std::sin(yaw);
    blocked_.assign(cells.size(), false);
    // Cover both the occupied-cell and queried-cell quantization error.
    const double radius=clearance+std::sqrt(2.)*resolution;
    const int span=static_cast<int>(std::ceil(radius/resolution));
    std::vector<std::pair<int,int>> offsets;
    for(int y=-span;y<=span;++y)for(int x=-span;x<=span;++x)
      if(std::hypot(x*resolution,y*resolution)<=radius)offsets.emplace_back(x,y);
    for(unsigned y=0;y<height;++y)for(unsigned x=0;x<width;++x) {
      const auto cost=cells[size_t(y)*width+x];
      if(cost>=0 && cost<65)continue;
      for(const auto & d:offsets) {
        const int ix=int(x)+d.first,iy=int(y)+d.second;
        if(ix>=0 && iy>=0 && ix<int(width) && iy<int(height))
          blocked_[size_t(iy)*width+ix]=true;
      }
    }
  }
  bool clear(double x,double y)const
  {
    if(!std::isfinite(x+y) || blocked_.empty())return false;
    const double dx=x-ox_,dy=y-oy_;
    const double gx=(cosine_*dx+sine_*dy)/resolution_;
    const double gy=(-sine_*dx+cosine_*dy)/resolution_;
    if(gx<0 || gy<0 || gx>=width_ || gy>=height_)return false;
    const auto ix=static_cast<unsigned>(gx),iy=static_cast<unsigned>(gy);
    return !blocked_[size_t(iy)*width_+ix];
  }
private:
  unsigned width_{0},height_{0};
  double resolution_{0},ox_{0},oy_{0},cosine_{1},sine_{0};
  std::vector<bool> blocked_;
};
}  // namespace bot_navigation
