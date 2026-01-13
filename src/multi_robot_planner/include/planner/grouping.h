#pragma once
#include "planner/types.h"

namespace planner {

class Grouper {
public:
  Grouper(double crop_radius) : crop_r_(crop_radius) {}
  // Implements Algorithm 2 (time-indexed grouping with cropping window)
  std::vector<GroupAssignment> assign(const std::vector<RefTraj>& refs,
                                      const CorridorTimeline& sc) const;
private:
  double crop_r_;
};

} // namespace planner
