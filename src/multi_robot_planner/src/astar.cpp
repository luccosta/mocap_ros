#include "planner/astar.h"
#include <queue>
#include <unordered_map>
#include <cmath>

namespace planner {

struct Node { int ix, iy; double g, f; int parent_id=-1; };
static inline int idxy(int ix, int iy, int nx) { return iy*nx + ix; }

std::vector<Vec2> AStarPlanner::plan(const Vec2& start_w, const Vec2& goal_w) {
  const auto& m = world_->map;
  int nx = std::ceil((m.x_max-m.x_min)/m.res)+1;
  int ny = std::ceil((m.y_max-m.y_min)/m.res)+1;
  auto to_grid = [&](const Vec2& p){
    int ix = std::clamp((int)std::round((p.x()-m.x_min)/m.res),0,nx-1);
    int iy = std::clamp((int)std::round((p.y()-m.y_min)/m.res),0,ny-1);
    return std::pair<int,int>{ix,iy};
  };
  auto to_world = [&](int ix, int iy){
    return Vec2(m.x_min + ix*m.res, m.y_min + iy*m.res);
  };
  auto [sx,sy] = to_grid(start_w);
  auto [gx,gy] = to_grid(goal_w);

  std::vector<char> closed(nx*ny, 0);
  std::vector<Node> nodes; nodes.reserve(nx*ny);
  auto cmp=[&](int a,int b){return nodes[a].f>nodes[b].f;};
  std::priority_queue<int,std::vector<int>,decltype(cmp)> open(cmp);

  auto add_node = [&](int ix,int iy,double g,int parent){
    Node n; n.ix=ix;n.iy=iy;n.g=g; Vec2 pw=to_world(ix,iy);
    n.f = g + (to_world(gx,gy)-pw).norm();
    n.parent_id = parent; nodes.push_back(n); int id=nodes.size()-1; open.push(id); return id; };

  add_node(sx,sy,0.0,-1);
  std::unordered_map<int,int> best; best[idxy(sx,sy,nx)]=0;

  const int dx[8]={1,-1,0,0,1,1,-1,-1};
  const int dy[8]={0,0,1,-1,1,-1,1,-1};

  int goal_id=-1;
  while(!open.empty()){
    int id = open.top(); open.pop();
    auto n = nodes[id];
    int key = idxy(n.ix,n.iy,nx);
    if (closed[key]) continue; closed[key]=1;
    if (n.ix==gx && n.iy==gy){ goal_id=id; break; }
    for(int k=0;k<8;++k){
      int nxg=n.ix+dx[k], nyg=n.iy+dy[k];
      if (nxg<0||nxg>=nx||nyg<0||nyg>=ny) continue;
      Vec2 a=to_world(n.ix,n.iy), b=to_world(nxg,nyg);
      if (!world_->segmentFree(a,b)) continue;
      double step = (k<4? m.res : m.res*std::sqrt(2.0));
      double gnew = n.g + step;
      int key2=idxy(nxg,nyg,nx);
      if (!best.count(key2) || gnew<nodes[best[key2]].g){
        int nid = add_node(nxg,nyg,gnew,id); best[key2]=nid;
      }
    }
  }

  std::vector<Vec2> path;
  if (goal_id<0) return path;
  for(int id=goal_id; id!=-1; id=nodes[id].parent_id) {
    path.push_back(to_world(nodes[id].ix,nodes[id].iy));
  }
  std::reverse(path.begin(), path.end());
  return path;
}

} // namespace planner
