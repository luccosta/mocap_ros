// =========================================
// src/ifopt_problem.cpp
// =========================================
#include "planner/ifopt_problem.h"
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

#include <cassert>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace planner {

TrajectoryVars::~TrajectoryVars() = default;

// ===== TrajectoryVars implementation
TrajectoryVars::TrajectoryVars(const std::string& name,
                               const std::vector<int>& robots,
                               int H,
                               const std::map<int, RefTraj>& ref_by_robot,
                               const DynParams& dyn,
                               const std::map<int, RobotParams>& rp)
  // Build with the correct size so IFOPT/Ipopt see n immediately
  : ifopt::VariableSet(static_cast<int>(robots.size()) * ((H+1)*3 + H*2), name),
    robots_(robots), H_(H), dyn_(dyn), ref_by_robot_(ref_by_robot), rp_(rp)
{
  // Allocate decision vector
  x_.setZero(GetRows());

  // Layout base indices per robot
  const int n_per_robot = (H_+1)*3 + H_*2;
  int base = 0;
  for (int r : robots_) {
    r_base_[r] = base;
    base += n_per_robot;
  }

  // Initial guess from reference (states), zero controls
  for (int r : robots_) {
    const auto& rt = ref_by_robot_.at(r);
    for (int k = 0; k <= H_; ++k) {
      x_(idx_x(r,k))  = rt.states[k].x();
      x_(idx_y(r,k))  = rt.states[k].y();
      x_(idx_th(r,k)) = rt.states[k].z();
    }
    for (int k = 0; k < H_; ++k) {
      x_(idx_v(r,k)) = 0.0;
      x_(idx_w(r,k)) = 0.0;
    }
  }
}

int TrajectoryVars::idx_x(int r, int k) const { return r_base_.at(r) + k*3 + 0; }
int TrajectoryVars::idx_y(int r, int k) const { return r_base_.at(r) + k*3 + 1; }
int TrajectoryVars::idx_th(int r, int k) const { return r_base_.at(r) + k*3 + 2; }
int TrajectoryVars::idx_v(int r, int k) const { return r_base_.at(r) + (H_+1)*3 + k*2 + 0; }
int TrajectoryVars::idx_w(int r, int k) const { return r_base_.at(r) + (H_+1)*3 + k*2 + 1; }

ifopt::Component::VecBound TrajectoryVars::GetBounds() const {
  const double inf = 1e20;
  const double eps = 1e-6; // prevent collapsed bounds (fixed vars)
  ifopt::Component::VecBound b(x_.size());
  for (int i = 0; i < x_.size(); ++i) b[i] = ifopt::Bounds(-inf, +inf);

  for (int r : robots_) {
    const auto& prm = rp_.at(r);
    for (int k = 0; k < H_; ++k) {
      double vmin = prm.v_min, vmax = prm.v_max;
      if (std::abs(vmax - vmin) < eps) vmax = vmin + eps;
      double wmin = prm.w_min, wmax = prm.w_max;
      if (std::abs(wmax - wmin) < eps) wmax = wmin + eps;
      b[idx_v(r,k)] = ifopt::Bounds(vmin, vmax);
      b[idx_w(r,k)] = ifopt::Bounds(wmin, wmax);
    }
  }
  return b;
}

// ===== DynamicsConstraint
DynamicsConstraint::DynamicsConstraint(const std::shared_ptr<TrajectoryVars>& vars)
  : ifopt::ConstraintSet(
        static_cast<int>(vars->robots().size()) * vars->H() * 3, "dyn"),
    v_(vars) {}

Vec DynamicsConstraint::GetValues() const {
  Vec g = Vec::Zero(GetRows());
  int row = 0;
  const double dt = v_->dyn().dt;

  for (int r : v_->robots()) {
    for (int k = 0; k < v_->H(); ++k) {
      const double xk  = v_->GetValues()(v_->idx_x(r,k));
      const double yk  = v_->GetValues()(v_->idx_y(r,k));
      const double th  = v_->GetValues()(v_->idx_th(r,k));
      const double vln = v_->GetValues()(v_->idx_v(r,k));
      const double w   = v_->GetValues()(v_->idx_w(r,k));

      const double xk1  = v_->GetValues()(v_->idx_x(r,k+1));
      const double yk1  = v_->GetValues()(v_->idx_y(r,k+1));
      const double thk1 = v_->GetValues()(v_->idx_th(r,k+1));

      g(row++) = xk1 - (xk + dt * vln * std::cos(th));
      g(row++) = yk1 - (yk + dt * vln * std::sin(th));
      g(row++) = thk1 - (th + dt * w);
    }
  }
  return g;
}

ifopt::Component::VecBound DynamicsConstraint::GetBounds() const {
  ifopt::Component::VecBound b(GetRows());
  for (auto& bb : b) bb = ifopt::Bounds(0.0, 0.0);
  return b;
}

void DynamicsConstraint::FillJacobianBlock(std::string var_set,
                                           ifopt::Component::Jacobian& J) const {
  if (var_set != v_->GetName()) return;

  int row = 0;
  const double dt = v_->dyn().dt;

  for (int r : v_->robots()) {
    for (int k = 0; k < v_->H(); ++k) {
      const int ix  = v_->idx_x(r,k);
      const int iy  = v_->idx_y(r,k);
      const int ith = v_->idx_th(r,k);
      const int iv  = v_->idx_v(r,k);
      const int iw  = v_->idx_w(r,k);
      const int ix1  = v_->idx_x(r,k+1);
      const int iy1  = v_->idx_y(r,k+1);
      const int ith1 = v_->idx_th(r,k+1);

      const double th  = v_->GetValues()(ith);
      const double vln = v_->GetValues()(iv);

      // gx: x_{k+1} - (x_k + dt v cos th)
      J.coeffRef(row, ix1) =  1.0;
      J.coeffRef(row, ix)  = -1.0;
      J.coeffRef(row, iv)  = -dt * std::cos(th);
      J.coeffRef(row, ith) = +dt * vln * std::sin(th); // corrected sign
      row++;

      // gy: y_{k+1} - (y_k + dt v sin th)
      J.coeffRef(row, iy1) =  1.0;
      J.coeffRef(row, iy)  = -1.0;
      J.coeffRef(row, iv)  = -dt * std::sin(th);
      J.coeffRef(row, ith) = -dt * vln * std::cos(th); // corrected sign
      row++;

      // gth: th_{k+1} - (th_k + dt w)
      J.coeffRef(row, ith1) =  1.0;
      J.coeffRef(row, ith)  = -1.0;
      J.coeffRef(row, iw)   = -dt;
      row++;
    }
  }
}

// ===== BoundaryConstraint
BoundaryConstraint::BoundaryConstraint(const std::shared_ptr<TrajectoryVars>& vars,
                                       const std::map<int, Vec3>& start,
                                       const std::map<int, Vec3>& goal,
                                       int t_start, int t_end)
  : ifopt::ConstraintSet(static_cast<int>(vars->robots().size()) * 3 * 2,
                         "boundary"),
    v_(vars), start_(start), goal_(goal), t_start_(t_start), t_end_(t_end) {}

Vec BoundaryConstraint::GetValues() const {
  Vec g = Vec::Zero(GetRows());
  int row = 0;
  for (int r : v_->robots()) {
    // at t_start
    g(row++) = v_->GetValues()(v_->idx_x(r,t_start_))  - start_.at(r).x();
    g(row++) = v_->GetValues()(v_->idx_y(r,t_start_))  - start_.at(r).y();
    g(row++) = v_->GetValues()(v_->idx_th(r,t_start_)) - start_.at(r).z();
    // at t_end
    g(row++) = v_->GetValues()(v_->idx_x(r,t_end_))  - goal_.at(r).x();
    g(row++) = v_->GetValues()(v_->idx_y(r,t_end_))  - goal_.at(r).y();
    g(row++) = v_->GetValues()(v_->idx_th(r,t_end_)) - goal_.at(r).z();
  }
  return g;
}

ifopt::Component::VecBound BoundaryConstraint::GetBounds() const {
  ifopt::Component::VecBound b(GetRows());
  for (auto& bb : b) bb = ifopt::Bounds(0.0, 0.0);
  return b;
}

void BoundaryConstraint::FillJacobianBlock(std::string var_set,
                                           ifopt::Component::Jacobian& J) const {
  if (var_set != v_->GetName()) return;

  int row = 0;
  for (int r : v_->robots()) {
    J.coeffRef(row++, v_->idx_x(r,t_start_))  = 1.0;
    J.coeffRef(row++, v_->idx_y(r,t_start_))  = 1.0;
    J.coeffRef(row++, v_->idx_th(r,t_start_)) = 1.0;

    J.coeffRef(row++, v_->idx_x(r,t_end_))  = 1.0;
    J.coeffRef(row++, v_->idx_y(r,t_end_))  = 1.0;
    J.coeffRef(row++, v_->idx_th(r,t_end_)) = 1.0;
  }
}

// ===== CorridorConstraint
CorridorConstraint::CorridorConstraint(
    const std::shared_ptr<TrajectoryVars>& vars,
    const std::map<int, std::vector<PolyCorridor>>& boxes_by_robot)
  : ifopt::ConstraintSet(0, "corridor"),
    v_(vars), boxes_(boxes_by_robot) {
  int rows = 0;
  for (const auto& kv : boxes_) {
    const auto& seq = kv.second; // size H+1
    for (const auto& pc : seq) rows += static_cast<int>(pc.A.rows());
  }
  rows_ = rows;
  SetRows(rows_);
}

Vec CorridorConstraint::GetValues() const {
  Vec g = Vec::Zero(rows_);
  int row = 0;

  for (int r : v_->robots()) {
    const auto& seq = boxes_.at(r);
    for (int k = 0; k <= v_->H(); ++k) {
      const auto& pc = seq[k];
      Eigen::Vector2d p(v_->GetValues()(v_->idx_x(r,k)),
                        v_->GetValues()(v_->idx_y(r,k)));
      g.segment(row, pc.A.rows()) = pc.A * p - pc.b; // <= 0
      row += static_cast<int>(pc.A.rows());
    }
  }
  return g;
}

ifopt::Component::VecBound CorridorConstraint::GetBounds() const {
  const double inf = 1e20;
  ifopt::Component::VecBound b(rows_);
  for (auto& bb : b) bb = ifopt::Bounds(-inf, 0.0);
  return b;
}

void CorridorConstraint::FillJacobianBlock(std::string var_set,
                                           ifopt::Component::Jacobian& J) const {
  if (var_set != v_->GetName()) return;

  int row = 0;
  for (int r : v_->robots()) {
    const auto& seq = boxes_.at(r);
    for (int k = 0; k <= v_->H(); ++k) {
      const auto& pc = seq[k];
      // Only x,y affect Ax-b
      for (int i = 0; i < pc.A.rows(); ++i) {
        J.coeffRef(row + i, v_->idx_x(r,k)) = pc.A(i,0);
        J.coeffRef(row + i, v_->idx_y(r,k)) = pc.A(i,1);
      }
      row += static_cast<int>(pc.A.rows());
    }
  }
}

// ===== SeparationConstraint
SeparationConstraint::SeparationConstraint(
    const std::shared_ptr<TrajectoryVars>& vars,
    const std::vector<std::pair<int,int>>& pairs,
    double min_dist)
  : ifopt::ConstraintSet(static_cast<int>(pairs.size()) * (vars->H()+1),
                         "separation"),
    v_(vars), pairs_(pairs), dmin2_(min_dist*min_dist) {}

Vec SeparationConstraint::GetValues() const {
  Vec g = Vec::Zero(GetRows());
  int row = 0;

  for (auto [a, b] : pairs_) {
    for (int k = 0; k <= v_->H(); ++k) {
      const double dx = v_->GetValues()(v_->idx_x(a,k)) - v_->GetValues()(v_->idx_x(b,k));
      const double dy = v_->GetValues()(v_->idx_y(a,k)) - v_->GetValues()(v_->idx_y(b,k));
      const double d2 = dx*dx + dy*dy;
      g(row++) = dmin2_ - d2; // <= 0
    }
  }
  return g;
}

ifopt::Component::VecBound SeparationConstraint::GetBounds() const {
  const double inf = 1e20;
  ifopt::Component::VecBound b(GetRows());
  for (auto& bb : b) bb = ifopt::Bounds(-inf, 0.0);
  return b;
}

void SeparationConstraint::FillJacobianBlock(std::string var_set,
                                             ifopt::Component::Jacobian& J) const {
  if (var_set != v_->GetName()) return;

  int row = 0;
  for (auto [a, b] : pairs_) {
    for (int k = 0; k <= v_->H(); ++k) {
      const double dx = v_->GetValues()(v_->idx_x(a,k)) - v_->GetValues()(v_->idx_x(b,k));
      const double dy = v_->GetValues()(v_->idx_y(a,k)) - v_->GetValues()(v_->idx_y(b,k));
      J.coeffRef(row, v_->idx_x(a,k)) = -2.0 * dx;
      J.coeffRef(row, v_->idx_y(a,k)) = -2.0 * dy;
      J.coeffRef(row, v_->idx_x(b,k)) =  2.0 * dx;
      J.coeffRef(row, v_->idx_y(b,k)) =  2.0 * dy;
      row++;
    }
  }
}

// ===== TrackingCost
TrackingCost::TrackingCost(const std::shared_ptr<TrajectoryVars>& vars,
                           double w_xy, double w_th)
  : ifopt::CostTerm("track"), v_(vars), w_xy_(w_xy), w_th_(w_th) {}

double TrackingCost::GetCost() const {
  double J = 0.0;
  for (int r : v_->robots()) {
    const auto& ref = v_->ref().at(r);
    for (int k = 0; k <= v_->H(); ++k) {
      const double dx  = v_->GetValues()(v_->idx_x(r,k))  - ref.states[k].x();
      const double dy  = v_->GetValues()(v_->idx_y(r,k))  - ref.states[k].y();
      const double dth = v_->GetValues()(v_->idx_th(r,k)) - ref.states[k].z();
      J += w_xy_ * (dx*dx + dy*dy) + w_th_ * (dth*dth);
    }
  }
  return J;
}

void TrackingCost::FillJacobianBlock(std::string var_set,
                                     ifopt::Component::Jacobian& J) const {
  if (var_set != v_->GetName()) return;

  for (int r : v_->robots()) {
    const auto& ref = v_->ref().at(r);
    for (int k = 0; k <= v_->H(); ++k) {
      const double dx  = v_->GetValues()(v_->idx_x(r,k))  - ref.states[k].x();
      const double dy  = v_->GetValues()(v_->idx_y(r,k))  - ref.states[k].y();
      const double dth = v_->GetValues()(v_->idx_th(r,k)) - ref.states[k].z();
      J.coeffRef(0, v_->idx_x(r,k))  += 2.0 * w_xy_ * dx;
      J.coeffRef(0, v_->idx_y(r,k))  += 2.0 * w_xy_ * dy;
      J.coeffRef(0, v_->idx_th(r,k)) += 2.0 * w_th_ * dth;
    }
  }
}

// ===== SmoothControlCost
SmoothControlCost::SmoothControlCost(const std::shared_ptr<TrajectoryVars>& vars,
                                     double w)
  : ifopt::CostTerm("smooth_u"), v_(vars), w_(w) {}

double SmoothControlCost::GetCost() const {
  double J = 0.0;
  for (int r : v_->robots()) {
    double pv = 0.0, pw = 0.0;
    for (int k = 0; k < v_->H(); ++k) {
      const double vln = v_->GetValues()(v_->idx_v(r,k));
      const double omg = v_->GetValues()(v_->idx_w(r,k));
      const double dv = vln - pv;
      const double dw = omg - pw;
      J += w_ * (dv*dv + dw*dw);
      pv = vln; pw = omg;
    }
  }
  return J;
}

void SmoothControlCost::FillJacobianBlock(std::string var_set,
                                          ifopt::Component::Jacobian& J) const {
  if (var_set != v_->GetName()) return;

  for (int r : v_->robots()) {
    double pv = 0.0, pw = 0.0;
    for (int k = 0; k < v_->H(); ++k) {
      const double vln = v_->GetValues()(v_->idx_v(r,k));
      const double omg = v_->GetValues()(v_->idx_w(r,k));
      const double dv = vln - pv;
      const double dw = omg - pw;
      J.coeffRef(0, v_->idx_v(r,k)) += 2.0 * w_ * dv;
      J.coeffRef(0, v_->idx_w(r,k)) += 2.0 * w_ * dw;
      pv = vln; pw = omg;
    }
  }
}

// ===== Solve one stage
StagePlan solve_stage(const std::vector<int>& robots,
                      int t_start, int t_end,
                      const std::map<int, RefTraj>& ref_by_robot,
                      const std::map<int, RobotParams>& rparams,
                      const std::map<int, std::vector<PolyCorridor>>& boxes,
                      const ProblemWeights& w,
                      const DynParams& dyn)
{
  // Slice references
  std::map<int, RefTraj> ref_slice;
  std::map<int, Vec3> start, goal;
  for (int r : robots) {
    const auto& rt = ref_by_robot.at(r);
    RefTraj s;
    s.t.reserve(t_end - t_start + 1);
    s.states.reserve(t_end - t_start + 1);
    for (int k = t_start; k <= t_end; ++k) {
      s.states.push_back(rt.states[k]);
      s.t.push_back(rt.t[k]);
    }
    ref_slice[r] = s;
    start[r] = rt.states[t_start];
    goal[r]  = rt.states[t_end];
  }

  const int H = t_end - t_start; // intervals
  auto vars = std::make_shared<TrajectoryVars>("traj", robots, H, ref_slice, dyn, rparams);

  ifopt::Problem nlp;
  nlp.AddVariableSet(vars);

  // Constraints
  nlp.AddConstraintSet(std::make_shared<DynamicsConstraint>(vars));
  nlp.AddConstraintSet(std::make_shared<BoundaryConstraint>(vars, start, goal, 0, H));

  // Corridors slice
  std::map<int, std::vector<PolyCorridor>> boxes_slice;
  for (int r : robots) {
    const auto& vec = boxes.at(r);
    std::vector<PolyCorridor> sub; sub.reserve(H+1);
    for (int k = t_start; k <= t_end; ++k) sub.push_back(vec[k]);
    boxes_slice[r] = std::move(sub);
  }
  nlp.AddConstraintSet(std::make_shared<CorridorConstraint>(vars, boxes_slice));

  // Separation for all pairs in "robots"
  std::vector<std::pair<int,int>> pairs;
  for (size_t i = 0; i < robots.size(); ++i)
    for (size_t j = i + 1; j < robots.size(); ++j)
      pairs.emplace_back(robots[i], robots[j]);

  if (!pairs.empty()) {
    double dmin = 0.0;
    for (auto [a,b] : pairs)
      dmin = std::max(dmin, rparams.at(a).radius + rparams.at(b).radius);
    nlp.AddConstraintSet(std::make_shared<SeparationConstraint>(vars, pairs, dmin));
  }

  // Costs
  nlp.AddCostSet(std::make_shared<TrackingCost>(vars, w.w_track_xy, w.w_track_th));
  nlp.AddCostSet(std::make_shared<SmoothControlCost>(vars, w.w_smooth_u));

  // --- Ipopt: clearer logs + robust defaults ---
  ifopt::IpoptSolver solver;
  solver.SetOption("print_level", 5); // shows n, m, bounds, etc.
  solver.SetOption("tol", 1e-3);
  solver.SetOption("hessian_approximation", "limited-memory");
  solver.SetOption("linear_solver", "mumps"); // change if needed

  std::cout << "[ifopt] vars n = " << vars->GetRows() << std::endl;
  solver.Solve(nlp);

  // Extract states
  StagePlan out; out.t_start = t_start; out.t_end = t_end;
  for (int r : robots) {
    std::vector<Vec3> xs; xs.reserve(H+1);
    for (int k = 0; k <= H; ++k) {
      xs.emplace_back(vars->GetValues()(vars->idx_x(r,k)),
                      vars->GetValues()(vars->idx_y(r,k)),
                      vars->GetValues()(vars->idx_th(r,k)));
    }
    out.states_by_robot[r] = std::move(xs);
  }
  return out;
}

} // namespace planner
