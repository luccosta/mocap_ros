cmake_minimum_required(VERSION 3.10)
project(icp_pose_estimator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS io registration)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(pointcloud_to_poses src/pointcloud_to_poses.cpp)
ament_target_dependencies(pointcloud_to_poses
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  ament_index_cpp
)
target_link_libraries(pointcloud_to_poses
  ${PCL_LIBRARIES}
  yaml-cpp
)

install(TARGETS pointcloud_to_poses
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
