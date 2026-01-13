cmake_minimum_required(VERSION 3.14)
project(ifopt_multirobot_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ifopt REQUIRED)

include_directories(include)

# Core library (planner logic)
add_library(planner_core
  src/map.cpp
  src/astar.cpp
  src/safe_corridor.cpp
  src/grouping.cpp
  src/ifopt_problem.cpp)

target_link_libraries(planner_core
  Eigen3::Eigen
  ifopt::ifopt_core
  ifopt::ifopt_ipopt)

# ROS 2 node
add_executable(planner_node
  src/ros/planner_node.cpp)

target_link_libraries(planner_node planner_core)
ament_target_dependencies(planner_node
  rclcpp geometry_msgs nav_msgs std_srvs visualization_msgs tf2 tf2_geometry_msgs)

install(TARGETS planner_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(planner_core)
ament_package()
