#include "planner/types.h"
#include "planner/map.h"
#include "planner/astar.h"
#include "planner/safe_corridor.h"
#include "planner/grouping.h"
#include "planner/ifopt_problem.h"
#include <iostream>

using namespace planner;

static RefTraj to_ref_traj(const std::vector<Vec2>& path, double dt) {
  RefTraj r; if (path.empty()) return r;
  for(size_t i=0;i<path.size();++i){
    Vec3 s; s.x()=path[i].x(); s.y()=path[i].y();
    if (i+1<path.size()){
      Vec2 d = path[i+1]-path[i]; s.z()=std::atan2(d.y(), d.x());
    } else if (i>0) {
      Vec2 d = path[i]-path[i-1]; s.z()=std::atan2(d.y(), d.x());
    } else s.z()=0.0;
    r.states.push_back(s); r.t.push_back(i*dt);
  }
  return r;
}

int main(){
  // 1) Define world: corridor with a central wall leaving a narrow passage
  World W; W.map.x_min=-4; W.map.x_max=4; W.map.y_min=-3; W.map.y_max=3; W.map.res=0.2;
  // vertical wall with a door
  W.map.obstacles.push_back({-0.2,-3,0.2,-0.6});
  W.map.obstacles.push_back({-0.2,0.6,0.2,3});

  // 2) Robots
  std::vector<RobotSpec> robots;
  robots.push_back({"R0", RobotParams{}, Vec3(-3.0, 0.0, 0.0), Vec3( 3.0, 0.0, 0.0)});
  robots.push_back({"R1", RobotParams{}, Vec3( 3.0, 0.2, 3.14), Vec3(-3.0, 0.2, 3.14)});

  DynParams dyn; dyn.dt = 0.2; dyn.H = 40; // 8s horizon

  // 3) Graph search references
  AStarPlanner astar(&W);
  std::vector<RefTraj> refs;
  for (const auto& rb : robots){
    auto p2 = astar.plan(rb.start.head<2>(), rb.goal.head<2>());
    if (p2.empty()) { std::cerr << "A* failed for " << rb.name << "\n"; return 1; }
    refs.push_back(to_ref_traj(p2, dyn.dt));
  }

  // 4) Safe corridors
  SafeCorridorBuilder scb(&W, /*width*/0.8);
  CorridorTimeline sc = scb.build(refs);

  // 5) Grouping (placeholder groups whole horizon together)
  Grouper grouper(/*crop_radius*/0.6);
  auto groups = grouper.assign(refs, sc);

  // 6) Build maps for solver
  std::map<int, RefTraj> ref_by_robot; std::map<int, RobotParams> rparams;
  std::map<int, std::vector<PolyCorridor>> boxes;
  for (int i=0;i<(int)robots.size();++i){ ref_by_robot[i]=refs[i]; rparams[i]=robots[i].params; boxes[i]=sc.corridors[i]; }

  ProblemWeights w; DynParams d = dyn;

  // 7) Multi-stage optimization (here single stage from groups[0])
  if (groups.empty()){ std::cerr << "No groups generated" << std::endl; return 1; }
  const auto& g = groups.front();
  StagePlan plan = solve_stage(g.robots, g.t_start, g.t_end, ref_by_robot, rparams, boxes, w, d);

  // 8) Print final states for inspection
  for (auto& [rid, xs] : plan.states_by_robot){
    std::cout << robots[rid].name << ":\n";
    for (size_t k=0;k<xs.size();++k){
      std::cout << k << ": (" << xs[k].x() << ", " << xs[k].y() << ", th=" << xs[k].z() << ")\n";
    }
  }
  return 0;
}
