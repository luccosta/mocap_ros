#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <map>
#include <optional>

namespace planner {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat = Eigen::MatrixXd;
using Vec = Eigen::VectorXd;

struct RobotParams {
  double radius = 0.25;   // [m]
  double v_min = -0.8;    // [m/s]
  double v_max =  0.8;
  double w_min = -1.5;    // [rad/s]
  double w_max =  1.5;
};

struct PolyCorridor { // A x <= b for [x y]^T
  Eigen::Matrix<double, Eigen::Dynamic, 2> A;
  Eigen::VectorXd b;
};

struct CorridorTimeline { // per time step
  // at each step k, a poly corridor for each robot i
  // corridors[i][k]
  std::vector<std::vector<PolyCorridor>> corridors;
};

struct RefTraj { // reference waypoints (length H+1)
  std::vector<Vec3> states; // [x,y,theta]
  std::vector<double> t;    // time stamps matching states
};

struct RobotSpec {
  std::string name;
  RobotParams params;
  Vec3 start; // [x y th]
  Vec3 goal;
};

struct GroupAssignment { // element e = ((t1..tm), (i1..in))
  int t_start; // inclusive index in 0..H
  int t_end;   // inclusive
  std::vector<int> robots; // indices into robots vector
};

struct StagePlan {
  // Optimized states for robots over [t_start..t_end]
  int t_start = 0, t_end = 0;
  std::map<int, std::vector<Vec3>> states_by_robot;
};

struct ProblemWeights {
  double w_track_xy = 10.0;
  double w_track_th = 1.0;
  double w_smooth_u = 1.0;
};

struct DynParams {
  double dt = 0.2; // [s]
  int H = 40;      // horizon length (# intervals), states H+1
};

} // namespace planner
