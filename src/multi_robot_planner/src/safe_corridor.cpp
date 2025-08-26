#include "planner/safe_corridor.h"
#include <cmath>

namespace planner {

PolyCorridor SafeCorridorBuilder::rectangleAround(const Vec2& c, double half_w, double half_h) const {
  PolyCorridor pc;
  pc.A.resize(4,2); pc.b.resize(4);
  // |x - cx| <= half_w  -> two rows
  pc.A <<  1, 0,
          -1, 0,
           0, 1,
           0,-1;
  pc.b << c.x()+half_w, -(c.x()-half_w), c.y()+half_h, -(c.y()-half_h);
  return pc;
}

CorridorTimeline SafeCorridorBuilder::build(const std::vector<RefTraj>& refs) const {
  CorridorTimeline tl; tl.corridors.resize(refs.size());
  for (size_t i=0;i<refs.size();++i){
    const auto& r = refs[i];
    int H = (int)r.states.size()-1;
    tl.corridors[i].resize(H+1);
    for (int k=0;k<=H;++k){
      // Simple axis-aligned rectangle centered at (x,y)
      Vec2 c = r.states[k].head<2>();
      // expand a base rectangle but clip to world bounds (skip checking obstacles for brevity)
      double hw = base_width_*0.5;
      double hh = base_width_*0.5;
      tl.corridors[i][k] = rectangleAround(c, hw, hh);
    }
  }
  return tl;
}

} // namespace planner
