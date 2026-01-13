#pragma once
#include "planner/map.h"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

namespace planner::ros_adapter {

inline void fromOccupancyGrid(const nav_msgs::msg::OccupancyGrid& msg, GridMap& out){
  out.occ.has_grid = true;
  out.occ.width = msg.info.width;
  out.occ.height = msg.info.height;
  out.occ.res = msg.info.resolution;
  out.occ.origin_x = msg.info.origin.position.x;
  out.occ.origin_y = msg.info.origin.position.y;
  out.occ.data.assign(msg.data.begin(), msg.data.end());
}

inline Vec3 toState(const geometry_msgs::msg::Pose& p){
  // yaw from quaternion
  double x=p.position.x, y=p.position.y;
  double qw=p.orientation.w, qx=p.orientation.x, qy=p.orientation.y, qz=p.orientation.z;
  double siny_cosp = 2*(qw*qz + qx*qy);
  double cosy_cosp = 1 - 2*(qy*qy + qz*qz);
  double yaw = std::atan2(siny_cosp, cosy_cosp);
  return Vec3(x,y,yaw);
}

inline std::vector<Vec3> posesToStates(const std::vector<geometry_msgs::msg::Pose>& poses){
  std::vector<Vec3> out; out.reserve(poses.size());
  for (auto& p : poses) out.push_back(toState(p));
  return out;
}

}
