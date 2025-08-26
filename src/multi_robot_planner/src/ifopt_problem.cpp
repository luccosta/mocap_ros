#include "planner/ifopt_problem.h"
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>
#include <cassert>
#include <cmath>

namespace planner {

TrajectoryVars::~TrajectoryVars() = default;

// ===== TrajectoryVars implementation
TrajectoryVars::TrajectoryVars(const std::string& name,
                 const std::vector<int>& robots,
                 int H,
                 const std::map<int, RefTraj>& ref_by_robot,
                 const DynParams& dyn,
                 const std::map<int, RobotParams>& rp)
: ifopt::VariableSet(0, name), robots_(robots), H_(H), dyn_(dyn), ref_by_robot_(ref_by_robot), rp_(rp)
{
  // compute size
  int n_per_robot = (H_+1)*3 + H_*2; // states + controls
  int total = (int)robots_.size()*n_per_robot;
  x_.setZero(total);
  // layout base indices
  int base=0; 
  for(int r : robots_) { r_base_[r]=base; base+=n_per_robot; }
  // initial guess from ref
  for(int r : robots_) {
    const auto& rt = ref_by_robot_.at(r);
    for(int k=0;k<=H_;++k){
      x_(idx_x(r,k)) = rt.states[k].x();
      x_(idx_y(r,k)) = rt.states[k].y();
      x_(idx_th(r,k))= rt.states[k].z();
    }
    for(int k=0;k<H_;++k){
      x_(idx_v(r,k)) = 0.0; x_(idx_w(r,k)) = 0.0; // zero controls as guess
    }
  }
  // set number of variables
  SetRows(x_.size());
}

ifopt::Component::VecBound TrajectoryVars::GetBounds() const
{
  VecBound bounds(GetRows());
  bounds.at(0) = ifopt::NoBound;
  bounds.at(1) = ifopt::NoBound;
  return bounds;
}


int TrajectoryVars::idx_x(int r, int k) const { return r_base_.at(r) + k*3 + 0; }
int TrajectoryVars::idx_y(int r, int k) const { return r_base_.at(r) + k*3 + 1; }
int TrajectoryVars::idx_th(int r, int k) const { return r_base_.at(r) + k*3 + 2; }
int TrajectoryVars::idx_v(int r, int k) const { return r_base_.at(r) + (H_+1)*3 + k*2 + 0; }
int TrajectoryVars::idx_w(int r, int k) const { return r_base_.at(r) + (H_+1)*3 + k*2 + 1; }

// ===== DynamicsConstraint
DynamicsConstraint::DynamicsConstraint(const std::shared_ptr<TrajectoryVars>& vars)
: ifopt::ConstraintSet(0, "dyn"), v_(vars) {
  // equality constraints: 3 per (r,k) for k=0..H-1
  int n = (int)v_->robots().size() * v_->H() * 3;
  SetRows(n);
}

Vec DynamicsConstraint::GetValues() const {
  Vec g = Vec::Zero(GetRows());
  int row=0;
  double dt = v_->dyn().dt;
  for(int r : v_->robots()){
    for(int k=0;k<v_->H();++k){
      double xk = v_->GetValues()(v_->idx_x(r,k));
      double yk = v_->GetValues()(v_->idx_y(r,k));
      double th = v_->GetValues()(v_->idx_th(r,k));
      double vlin = v_->GetValues()(v_->idx_v(r,k));
      double w = v_->GetValues()(v_->idx_w(r,k));

      double xkp1 = v_->GetValues()(v_->idx_x(r,k+1));
      double ykp1 = v_->GetValues()(v_->idx_y(r,k+1));
      double thp1 = v_->GetValues()(v_->idx_th(r,k+1));

      g(row++) = xkp1 - (xk + dt*vlin*std::cos(th));
      g(row++) = ykp1 - (yk + dt*vlin*std::sin(th));
      g(row++) = thp1 - (th + dt*w);
    }
  }
  return g;
}

void DynamicsConstraint::FillJacobianBlock(std::string var_set, ifopt::Component::Jacobian& J) const {
  if (var_set != v_->GetName()) return;
  int row=0; double dt = v_->dyn().dt;
  for(int r : v_->robots()){
    for(int k=0;k<v_->H();++k){
      int ix = v_->idx_x(r,k); int iy = v_->idx_y(r,k); int ith = v_->idx_th(r,k);
      int iv = v_->idx_v(r,k); int iw = v_->idx_w(r,k);
      int ix1 = v_->idx_x(r,k+1); int iy1 = v_->idx_y(r,k+1); int ith1 = v_->idx_th(r,k+1);
      double th = v_->GetValues()(ith);
      double vlin = v_->GetValues()(iv);
      // Row for x
      J.coeffRef(row, ix1) = 1.0; 
      J.coeffRef(row, ix) = -1.0;
      J.coeffRef(row, ith) = -dt*vlin*std::sin(th);
      J.coeffRef(row, iv)  = -dt*std::cos(th);
      row++;
      // Row for y
      J.coeffRef(row, iy1) = 1.0; 
      J.coeffRef(row, iy) = -1.0;
      J.coeffRef(row, ith) =  dt*vlin*std::cos(th);
      J.coeffRef(row, iv)  = -dt*std::sin(th);
      row++;
      // Row for theta
      J.coeffRef(row, ith1) = 1.0; 
      J.coeffRef(row, ith) = -1.0;
      J.coeffRef(row, iw)   = -dt;
      row++;
    }
  }
}

ifopt::Component::VecBound DynamicsConstraint::GetBounds() const 
{
  ifopt::Component::VecBound bounds(GetRows());
  bounds.at(0) = ifopt::NoBound;
  bounds.at(1) = ifopt::NoBound;
  return bounds;
}

// ===== BoundaryConstraint
BoundaryConstraint::BoundaryConstraint(const std::shared_ptr<TrajectoryVars>& vars,
                     const std::map<int, Vec3>& start,
                     const std::map<int, Vec3>& goal,
                     int t_start, int t_end)
: ifopt::ConstraintSet(0, "boundary"), v_(vars), start_(start), goal_(goal), t_start_(t_start), t_end_(t_end)
{
  // equality: 3*(#robots)*2  (fix state at t_start and t_end)
  int n = (int)v_->robots().size()*3*2;
  SetRows(n);
}

Vec BoundaryConstraint::GetValues() const {
  Vec g = Vec::Zero(GetRows()); int row=0;
  for(int r : v_->robots()){
    // at t_start
    g(row++) = v_->GetValues()(v_->idx_x(r,t_start_)) - start_.at(r).x();
    g(row++) = v_->GetValues()(v_->idx_y(r,t_start_)) - start_.at(r).y();
    g(row++) = v_->GetValues()(v_->idx_th(r,t_start_)) - start_.at(r).z();
    // at t_end
    g(row++) = v_->GetValues()(v_->idx_x(r,t_end_)) - goal_.at(r).x();
    g(row++) = v_->GetValues()(v_->idx_y(r,t_end_)) - goal_.at(r).y();
    g(row++) = v_->GetValues()(v_->idx_th(r,t_end_)) - goal_.at(r).z();
  }
  return g;
}

void BoundaryConstraint::FillJacobianBlock(std::string var_set, ifopt::Component::Jacobian& J) const {
  if (var_set != v_->GetName()) return; int row=0;
  for(int r : v_->robots()){
    J.coeffRef(row++, v_->idx_x(r,t_start_)) = 1.0;
    J.coeffRef(row++, v_->idx_y(r,t_start_)) = 1.0;
    J.coeffRef(row++, v_->idx_th(r,t_start_))= 1.0;
    J.coeffRef(row++, v_->idx_x(r,t_end_))   = 1.0;
    J.coeffRef(row++, v_->idx_y(r,t_end_))   = 1.0;
    J.coeffRef(row++, v_->idx_th(r,t_end_))  = 1.0;
  }
}

ifopt::Component::VecBound BoundaryConstraint::GetBounds() const 
{
  ifopt::Component::VecBound bounds(GetRows());
  bounds.at(0) = ifopt::NoBound;
  bounds.at(1) = ifopt::NoBound;
  return bounds;
}

// ===== CorridorConstraint
CorridorConstraint::CorridorConstraint(const std::shared_ptr<TrajectoryVars>& vars,
                     const std::map<int, std::vector<PolyCorridor>>& boxes_by_robot)
: ifopt::ConstraintSet(0, "corridor"), v_(vars), boxes_(boxes_by_robot)
{
  int rows = 0;
  for (auto& kv : boxes_) {
    int i = kv.first; (void)i;
    const auto& vec = kv.second; // size H+1
    for (const auto& pc : vec) rows += pc.A.rows();
  }
  rows_ = rows; SetRows(rows_);
}

Vec CorridorConstraint::GetValues() const {
  Vec g = Vec::Zero(rows_); int row=0;
  for (int r : v_->robots()){
    const auto& vec = boxes_.at(r);
    for (int k=0;k<=v_->H();++k){
      Eigen::Vector2d p;
      p << v_->GetValues()(v_->idx_x(r,k)), v_->GetValues()(v_->idx_y(r,k));
      const auto& pc = vec[k];
      g.segment(row, pc.A.rows()) = pc.A * p - pc.b; // must be <= 0
      row += pc.A.rows();
    }
  }
  return g;
}

void CorridorConstraint::FillJacobianBlock(std::string var_set, ifopt::Component::Jacobian& J) const {
  if (var_set != v_->GetName()) return; 
  
  int row=0;
  for (int r : v_->robots()){
    const auto& vec = boxes_.at(r);
    for (int k=0;k<=v_->H();++k){
      const auto& pc = vec[k];
      // d(Ax-b)/dx = A  (only x,y columns active)
      for (int i=0;i<pc.A.rows();++i){
        J.coeffRef(row+i, v_->idx_x(r,k)) = pc.A(i,0);
        J.coeffRef(row+i, v_->idx_y(r,k)) = pc.A(i,1);
      }
      row += pc.A.rows();
    }
  }
}

ifopt::Component::VecBound CorridorConstraint::GetBounds() const 
{
  ifopt::Component::VecBound bounds(GetRows());
  bounds.at(0) = ifopt::NoBound;
  bounds.at(1) = ifopt::NoBound;
  return bounds;
}

// ===== SeparationConstraint
SeparationConstraint::SeparationConstraint(const std::shared_ptr<TrajectoryVars>& vars,
                       const std::vector<std::pair<int,int>>& pairs,
                       double min_dist)
: ifopt::ConstraintSet(0, "separation"), v_(vars), pairs_(pairs), dmin2_(min_dist*min_dist)
{
  int rows = (int)pairs_.size() * (v_->H()+1);
  SetRows(rows);
}

Vec SeparationConstraint::GetValues() const {
  Vec g = Vec::Zero(GetRows()); int row=0;
  for (auto [a,b] : pairs_){
    for(int k=0;k<=v_->H();++k){
      double dx = v_->GetValues()(v_->idx_x(a,k)) - v_->GetValues()(v_->idx_x(b,k));
      double dy = v_->GetValues()(v_->idx_y(a,k)) - v_->GetValues()(v_->idx_y(b,k));
      double d2 = dx*dx + dy*dy;
      g(row++) = dmin2_ - d2; // <= 0
    }
  }
  return g;
}

void SeparationConstraint::FillJacobianBlock(std::string var_set, ifopt::Component::Jacobian& J) const {
  if (var_set != v_->GetName()) return; int row=0;
  for (auto [a,b] : pairs_){
    for(int k=0;k<=v_->H();++k){
      double dx = v_->GetValues()(v_->idx_x(a,k)) - v_->GetValues()(v_->idx_x(b,k));
      double dy = v_->GetValues()(v_->idx_y(a,k)) - v_->GetValues()(v_->idx_y(b,k));
      J.coeffRef(row, v_->idx_x(a,k)) = -2.0*dx;
      J.coeffRef(row, v_->idx_y(a,k)) = -2.0*dy;
      J.coeffRef(row, v_->idx_x(b,k)) =  2.0*dx;
      J.coeffRef(row, v_->idx_y(b,k)) =  2.0*dy;
      row++;
    }
  }
}

ifopt::Component::VecBound SeparationConstraint::GetBounds() const 
{
  ifopt::Component::VecBound bounds(GetRows());
  bounds.at(0) = ifopt::NoBound;
  bounds.at(1) = ifopt::NoBound;
  return bounds;
}

// ===== TrackingCost
TrackingCost::TrackingCost(const std::shared_ptr<TrajectoryVars>& vars, double w_xy, double w_th)
: ifopt::CostTerm("track"), v_(vars), w_xy_(w_xy), w_th_(w_th) {}

double TrackingCost::GetCost() const {
  double J = 0.0;
  for(int r : v_->robots()){
    const auto& ref = v_->ref().at(r);
    for(int k=0;k<=v_->H();++k){
      double dx = v_->GetValues()(v_->idx_x(r,k)) - ref.states[k].x();
      double dy = v_->GetValues()(v_->idx_y(r,k)) - ref.states[k].y();
      double dth= v_->GetValues()(v_->idx_th(r,k)) - ref.states[k].z();
      J += w_xy_*(dx*dx + dy*dy) + w_th_*(dth*dth);
    }
  }
  return J;
}

void TrackingCost::FillJacobianBlock(std::string var_set, ifopt::Component::Jacobian& J) const {
  if (var_set != v_->GetName()) return;
  for(int r : v_->robots()){
    const auto& ref = v_->ref().at(r);
    for(int k=0;k<=v_->H();++k){
      double dx = v_->GetValues()(v_->idx_x(r,k)) - ref.states[k].x();
      double dy = v_->GetValues()(v_->idx_y(r,k)) - ref.states[k].y();
      double dth= v_->GetValues()(v_->idx_th(r,k)) - ref.states[k].z();
      J.coeffRef(0, v_->idx_x(r,k)) += 2.0*w_xy_*dx;
      J.coeffRef(0, v_->idx_y(r,k)) += 2.0*w_xy_*dy;
      J.coeffRef(0, v_->idx_th(r,k))+= 2.0*w_th_*dth;
    }
  }
}

// ===== SmoothControlCost
SmoothControlCost::SmoothControlCost(const std::shared_ptr<TrajectoryVars>& vars, double w)
: ifopt::CostTerm("smooth_u"), v_(vars), w_(w) {}

double SmoothControlCost::GetCost() const {
  double J=0.0;
  for(int r : v_->robots()){
    double pv=0.0,pw=0.0; // previous control (0)
    for(int k=0;k<v_->H();++k){
      double vlin = v_->GetValues()(v_->idx_v(r,k));
      double w    = v_->GetValues()(v_->idx_w(r,k));
      double dv = vlin-pv, dw=w-pw;
      J += w_*(dv*dv + dw*dw);
      pv=vlin; pw=w;
    }
  }
  return J;
}

void SmoothControlCost::FillJacobianBlock(std::string var_set, ifopt::Component::Jacobian& J) const {
  if (var_set != v_->GetName()) return;
  for(int r : v_->robots()){
    double pv=0.0,pw=0.0; // previous control
    for(int k=0;k<v_->H();++k){
      double vlin = v_->GetValues()(v_->idx_v(r,k));
      double w    = v_->GetValues()(v_->idx_w(r,k));
      double dv = vlin-pv, dw=w-pw;
      J.coeffRef(0, v_->idx_v(r,k)) += 2.0*w_*dv;
      J.coeffRef(0, v_->idx_w(r,k)) += 2.0*w_*dw;
      pv=vlin; pw=w;
    }
  }
}

StagePlan solve_stage(const std::vector<int>& robots,
                      int t_start, int t_end,
                      const std::map<int, RefTraj>& ref_by_robot,
                      const std::map<int, RobotParams>& rparams,
                      const std::map<int, std::vector<PolyCorridor>>& boxes,
                      const ProblemWeights& w,
                      const DynParams& dyn)
{
  // Slice references to [t_start..t_end]
  std::map<int, RefTraj> ref_slice;
  std::map<int, Vec3> start, goal;
  for(int r : robots){
    const auto& rt = ref_by_robot.at(r);
    RefTraj s; s.t.reserve(t_end-t_start+1); s.states.reserve(t_end-t_start+1);
    for(int k=t_start;k<=t_end;++k){ s.states.push_back(rt.states[k]); s.t.push_back(rt.t[k]); }
    ref_slice[r]=s; start[r]=rt.states[t_start]; goal[r]=rt.states[t_end];
  }

  int H = t_end - t_start; // intervals in this stage
  auto vars = std::make_shared<TrajectoryVars>("traj", robots, H, ref_slice, dyn, rparams);

  ifopt::Problem nlp;
  nlp.AddVariableSet(vars);

  // Constraints
  nlp.AddConstraintSet(std::make_shared<DynamicsConstraint>(vars));
  nlp.AddConstraintSet(std::make_shared<BoundaryConstraint>(vars, start, goal, 0, H));

  // Corridors slice
  std::map<int, std::vector<PolyCorridor>> boxes_slice;
  for(int r : robots){
    const auto& vec = boxes.at(r);
    std::vector<PolyCorridor> sub; sub.reserve(H+1);
    for(int k=t_start;k<=t_end;++k) sub.push_back(vec[k]);
    boxes_slice[r] = std::move(sub);
  }
  nlp.AddConstraintSet(std::make_shared<CorridorConstraint>(vars, boxes_slice));

  // Separation for all pairs in "robots"
  std::vector<std::pair<int,int>> pairs;
  for(size_t i=0;i<robots.size();++i) for(size_t j=i+1;j<robots.size();++j) pairs.emplace_back(robots[i], robots[j]);
  double min_dist = 0.0; // filled per-pair from rparams radii
  if (!pairs.empty()){
    // compute min sum of radii (conservative)
    double dmin = 0.0;
    for (auto [a,b] : pairs) dmin = std::max(dmin, rparams.at(a).radius + rparams.at(b).radius);
    nlp.AddConstraintSet(std::make_shared<SeparationConstraint>(vars, pairs, dmin));
  }

  // Costs
  nlp.AddCostSet(std::make_shared<TrackingCost>(vars, w.w_track_xy, w.w_track_th));
  nlp.AddCostSet(std::make_shared<SmoothControlCost>(vars, w.w_smooth_u));

  ifopt::IpoptSolver solver; solver.SetOption("print_level", 0);
  solver.SetOption("tol", 1e-3);
  solver.Solve(nlp);

  StagePlan out; out.t_start = t_start; out.t_end = t_end;
  for(int r : robots){
    std::vector<Vec3> xs; xs.reserve(H+1);
    for(int k=0;k<=H;++k){
      xs.emplace_back(vars->GetValues()(vars->idx_x(r,k)),
                      vars->GetValues()(vars->idx_y(r,k)),
                      vars->GetValues()(vars->idx_th(r,k)));
    }
    out.states_by_robot[r] = std::move(xs);
  }
  return out;
}

} // namespace planner
