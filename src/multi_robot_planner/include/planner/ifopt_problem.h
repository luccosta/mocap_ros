#pragma once
#include "planner/types.h"
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

namespace planner {

// ===== VariableSet: trajectory states and controls for a stage & robot set
class TrajectoryVars : public ifopt::VariableSet {
public:
  TrajectoryVars(const std::string& name,
                 const std::vector<int>& robots,
                 int H,
                 const std::map<int, RefTraj>& ref_by_robot,
                 const DynParams& dyn,
                 const std::map<int, RobotParams>& rp);

  virtual ~TrajectoryVars() override;

  void SetVariables(const Vec& x) override { x_ = x; }
  Vec GetValues() const override { return x_; }
  ifopt::Component::VecBound GetBounds() const override;

  // indexing helpers
  int idx_x(int r, int k) const; // x state index
  int idx_y(int r, int k) const;
  int idx_th(int r, int k) const;
  int idx_v(int r, int k) const; // control v at interval k (0..H-1)
  int idx_w(int r, int k) const;

  const std::vector<int>& robots() const { return robots_; }
  int H() const { return H_; }
  const DynParams& dyn() const { return dyn_; }
  const std::map<int, RefTraj>& ref() const { return ref_by_robot_; }
  const std::map<int, RobotParams>& rparams() const { return rp_; }

private:
  std::vector<int> robots_;
  int H_;
  DynParams dyn_;
  std::map<int, RefTraj> ref_by_robot_;
  std::map<int, RobotParams> rp_;
  // layout: for each robot r: states (H+1)*3 then controls H*2
  // x_ packs all robots
  Vec x_;
  mutable std::map<int, int> r_base_; // base index per robot
};

// ===== Constraints
class DynamicsConstraint : public ifopt::ConstraintSet {
public:
  DynamicsConstraint(const std::shared_ptr<TrajectoryVars>& vars);
  Vec GetValues() const override;
  ifopt::Component::VecBound GetBounds() const override;
  void FillJacobianBlock(std::string var_set, ifopt::Component::Jacobian& jac_block) const override;
private:
  std::shared_ptr<TrajectoryVars> v_;
};

class BoundaryConstraint : public ifopt::ConstraintSet {
public:
  BoundaryConstraint(const std::shared_ptr<TrajectoryVars>& vars,
                     const std::map<int, Vec3>& start,
                     const std::map<int, Vec3>& goal,
                     int t_start, int t_end);
  Vec GetValues() const override;
  ifopt::Component::VecBound GetBounds() const override;
  void FillJacobianBlock(std::string var_set, ifopt::Component::Jacobian& jac_block) const override;
private:
  std::shared_ptr<TrajectoryVars> v_;
  std::map<int, Vec3> start_, goal_;
  int t_start_, t_end_;
};

class CorridorConstraint : public ifopt::ConstraintSet {
public:
  CorridorConstraint(const std::shared_ptr<TrajectoryVars>& vars,
                     const std::map<int, std::vector<PolyCorridor>>& boxes_by_robot);
  Vec GetValues() const override;
  ifopt::Component::VecBound GetBounds() const override;
  void FillJacobianBlock(std::string var_set, ifopt::Component::Jacobian& jac_block) const override;
private:
  std::shared_ptr<TrajectoryVars> v_;
  // For each robot, per k in [0..H], a corridor A x <= b
  std::map<int, std::vector<PolyCorridor>> boxes_;
  int rows_ = 0;
};

class SeparationConstraint : public ifopt::ConstraintSet {
public:
  SeparationConstraint(const std::shared_ptr<TrajectoryVars>& vars,
                       const std::vector<std::pair<int,int>>& pairs,
                       double min_dist);
  Vec GetValues() const override;
  ifopt::Component::VecBound GetBounds() const override;
  void FillJacobianBlock(std::string var_set, ifopt::Component::Jacobian& jac_block) const override;
private:
  std::shared_ptr<TrajectoryVars> v_;
  std::vector<std::pair<int,int>> pairs_;
  double dmin2_;
};

// ===== Costs
class TrackingCost : public ifopt::CostTerm {
public:
  TrackingCost(const std::shared_ptr<TrajectoryVars>& vars, double w_xy, double w_th);
  double GetCost() const override;
  void FillJacobianBlock(std::string var_set, ifopt::Component::Jacobian& jac) const override;
private:
  std::shared_ptr<TrajectoryVars> v_;
  double w_xy_, w_th_;
};

class SmoothControlCost : public ifopt::CostTerm {
public:
  SmoothControlCost(const std::shared_ptr<TrajectoryVars>& vars, double w);
  double GetCost() const override;
  void FillJacobianBlock(std::string var_set, ifopt::Component::Jacobian& jac) const override;
private:
  std::shared_ptr<TrajectoryVars> v_;
  double w_;
};

// ===== Solve one stage
StagePlan solve_stage(const std::vector<int>& robots,
                      int t_start, int t_end,
                      const std::map<int, RefTraj>& ref_by_robot,
                      const std::map<int, RobotParams>& rparams,
                      const std::map<int, std::vector<PolyCorridor>>& boxes,
                      const ProblemWeights& w,
                      const DynParams& dyn);

} // namespace planner
