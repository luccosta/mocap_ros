#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "planner/types.h"
#include "planner/map.h"
#include "planner/astar.h"
#include "planner/safe_corridor.h"
#include "planner/grouping.h"
#include "planner/ifopt_problem.h"
#include "planner/ros_adapter.hpp"

using namespace planner;

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode() : Node("ifopt_multirobot_planner") {
    // Parameters
    declare_parameter("dt", 0.2);
    declare_parameter("H", 40);
    declare_parameter("corridor_width", 0.8);
    declare_parameter("robot_radius", 0.25);
    declare_parameter("v_max", 0.8);
    declare_parameter("w_max", 1.5);

    dyn_.dt = get_parameter("dt").as_double();
    dyn_.H  = get_parameter("H").as_int();
    corridor_width_ = get_parameter("corridor_width").as_double();

    // Subs & pubs
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 1,
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        ros_adapter::fromOccupancyGrid(*msg, world_.map);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Map updated: %dx%d", world_.map.occ.width, world_.map.occ.height);
      });

    start_sub_ = create_subscription<geometry_msgs::msg::PoseArray>("/initial_poses", 10,
      [this](geometry_msgs::msg::PoseArray::SharedPtr msg){ starts_ = msg->poses; });
    goal_sub_  = create_subscription<geometry_msgs::msg::PoseArray>("/goal_poses", 10,
      [this](geometry_msgs::msg::PoseArray::SharedPtr msg){ goals_  = msg->poses; });

    plan_srv_ = create_service<std_srvs::srv::Trigger>("/plan",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res){
        bool ok = planOnce();
        res->success = ok; res->message = ok? "Plan computed" : "Planning failed";
      });

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planner/combined_path", 10);
  }

private:
  bool planOnce(){
    if (starts_.size() == 0 || goals_.size() == 0 || starts_.size()!=goals_.size()){
      RCLCPP_WARN(get_logger(), "Need matching initial_poses and goal_poses");
      return false;
    }
    // Build robots vector
    std::vector<RobotSpec> robots; robots.reserve(starts_.size());
    RobotParams rp; rp.radius = get_parameter("robot_radius").as_double();
    rp.v_max = get_parameter("v_max").as_double(); rp.v_min = -rp.v_max;
    rp.w_max = get_parameter("w_max").as_double(); rp.w_min = -rp.w_max;

    for (size_t i=0;i<starts_.size();++i){
      Vec3 s = ros_adapter::toState(starts_[i]);
      Vec3 g = ros_adapter::toState(goals_[i]);
      robots.push_back({"R"+std::to_string(i), rp, s, g});
    }

    // References via A*
    AStarPlanner astar(&world_);
    std::vector<RefTraj> refs;
    for (const auto& rb : robots){
      auto p2 = astar.plan(rb.start.head<2>(), rb.goal.head<2>());
      if (p2.empty()) { RCLCPP_ERROR(get_logger(), "A* failed"); return false; }
      // to_ref_traj helper (copy from main.cpp)
      RefTraj r; for(size_t i=0;i<p2.size();++i){
        Vec3 s; s.x()=p2[i].x(); s.y()=p2[i].y();
        if (i+1<p2.size()){
          Vec2 d = p2[i+1]-p2[i]; s.z()=std::atan2(d.y(), d.x());
        } else if (i>0) {
          Vec2 d = p2[i]-p2[i-1]; s.z()=std::atan2(d.y(), d.x());
        } else s.z()=0.0; r.states.push_back(s); r.t.push_back(i*dyn_.dt);
      }
      refs.push_back(r);
    }

    // Corridors and grouping
    SafeCorridorBuilder scb(&world_, corridor_width_);
    CorridorTimeline sc = scb.build(refs);
    Grouper grouper(0.6);
    auto groups = grouper.assign(refs, sc);

    // Build maps for solver
    std::map<int, RefTraj> ref_by_robot; std::map<int, RobotParams> rparams;
    std::map<int, std::vector<PolyCorridor>> boxes;
    for (int i=0;i<(int)robots.size();++i){ ref_by_robot[i]=refs[i]; rparams[i]=robots[i].params; boxes[i]=sc.corridors[i]; }

    ProblemWeights w; // use defaults

    if (groups.empty()) return false;
    const auto& g = groups.front();
    StagePlan plan = solve_stage(g.robots, g.t_start, g.t_end, ref_by_robot, rparams, boxes, w, dyn_);

    // Publish a combined Path (concatenate per-robot as sequential segments)
    nav_msgs::msg::Path path; path.header.stamp = now(); path.header.frame_id = "map";
    for (auto& [rid, xs] : plan.states_by_robot){
      for (auto& s : xs){
        geometry_msgs::msg::PoseStamped ps; ps.header = path.header;
        ps.pose.position.x = s.x(); ps.pose.position.y = s.y(); ps.pose.orientation.w = 1.0;
        path.poses.push_back(ps);
      }
    }
    path_pub_->publish(path);
    RCLCPP_INFO(get_logger(), "Published path with %zu poses", path.poses.size());
    return true;
  }

  // Data
  World world_;
  DynParams dyn_;
  double corridor_width_ = 0.8;

  // ROS I/O
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr start_sub_, goal_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_srv_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  std::vector<geometry_msgs::msg::Pose> starts_, goals_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
