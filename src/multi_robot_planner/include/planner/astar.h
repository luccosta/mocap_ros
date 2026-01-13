#pragma once
#include "planner/types.h"
#include "planner/map.h"

namespace planner {

class AStarPlanner {
public:
  explicit AStarPlanner(const World* world) : world_(world) {}
  // plan on grid, return waypoints in world
  std::vector<Vec2> plan(const Vec2& start, const Vec2& goal);
private:
  const World* world_;
};

} // namespace planner
