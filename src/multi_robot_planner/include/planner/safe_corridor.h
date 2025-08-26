#pragma once
#include "planner/types.h"
#include "planner/map.h"

namespace planner {

// Build per-robot, per-time-step convex corridors around reference path.
class SafeCorridorBuilder {
public:
  SafeCorridorBuilder(const World* world, double width = 0.6)
    : world_(world), base_width_(width) {}

  // For each robot's reference trajectory, produce axis-aligned rectangles
  // around segments, expanded until touching obstacles (simple heuristic).
  // Returns: corridors[i][k] valid for k=0..H (H+1 states; we reuse k for position boxes)
  CorridorTimeline build(const std::vector<RefTraj>& refs) const;

private:
  const World* world_;
  double base_width_;
  PolyCorridor rectangleAround(const Vec2& p, double half_w, double half_h) const;
};

} // namespace planner
