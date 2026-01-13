#include "planner/map.h"
#include <algorithm>
#include <cmath>

namespace planner {

bool GridMap::isOccupied(const Vec2& p) const {
  if (occ.has_grid) {
    // Convert to grid indices
    int ix = (int)std::floor((p.x() - occ.origin_x)/occ.res);
    int iy = (int)std::floor((p.y() - occ.origin_y)/occ.res);
    return occ.cellOccupied(ix, iy);
  }
  // Rectangles fallback and bounds
  if (p.x() < x_min || p.x() > x_max || p.y() < y_min || p.y() > y_max) return true;
  for (const auto& o : obstacles) {
    if (p.x() >= o.x_min && p.x() <= o.x_max && p.y() >= o.y_min && p.y() <= o.y_max) return true;
  }
  return false;
}

bool World::segmentFree(const Vec2& a, const Vec2& b, double step) const {
  Vec2 d = b - a; double L = d.norm();
  int n = std::max(1, (int)std::ceil(L/step));
  for (int i=0;i<=n;++i) {
    Vec2 p = a + (double(i)/n)*d;
    if (map.isOccupied(p)) return false;
  }
  return true;
}

} // namespace planner
