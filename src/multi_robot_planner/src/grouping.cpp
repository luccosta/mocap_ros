#include "planner/grouping.h"
#include <set>

namespace planner {

std::vector<GroupAssignment> Grouper::assign(const std::vector<RefTraj>& refs,
                                             const CorridorTimeline& sc) const {
  const int N = (int)refs.size();
  const int H = (int)refs[0].states.size()-1;

  // For each t, find sets of robots whose distance < 2*crop_r_ (proxy for corridor overlap)
  std::vector<std::set<int>> groups_per_t(H+1);
  for(int t=0;t<=H;++t){
    std::set<int> G;
    for(int i=0;i<N;++i){ G.insert(i); }
    groups_per_t[t]=G; // one group with all robots for simplicity
  }

  // Merge consecutive time steps with same group (all robots). This is a placeholder
  std::vector<GroupAssignment> out;
  int t0=0; while(t0<=H){
    int t1=t0; while(t1+1<=H) { ++t1; }
    GroupAssignment ga; ga.t_start=t0; ga.t_end=t1; ga.robots.clear();
    for(int i=0;i<N;++i) ga.robots.push_back(i);
    out.push_back(ga);
    t0=t1+1;
  }
  return out;
}

} // namespace planner
