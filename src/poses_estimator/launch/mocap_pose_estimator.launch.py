from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pose_estimator_pkg = FindPackageShare("icp_pose_estimator")

    ld = LaunchDescription()

    config_file = PathJoinSubstitution(
        [pose_estimator_pkg,
        'config',
        'clouds_mobile_robots.yaml']
    )

    rviz_config_file = PathJoinSubstitution(
        ['/ros2_ws/src/poses_estimator',
        'config',
        'icp_visualization.rviz']
    )

    mocap_driver = Node(
        package="mocap_ros_driver",
        executable='mocap_ros_driver',
        name='mocap_udp_node',
        output='screen'
    )
    ld.add_action(mocap_driver)

    poses_estimator = Node(
        package="icp_pose_estimator",
        executable='pointcloud_to_poses',
        name='icp_node',
        output='screen',
        parameters=[
            {'clouds_file_path': config_file},
            {'icp_max_correspondence': 0.5},
            {'icp_transformation_epsilon': 10.0},
            {'icp_euclidian_fitness_epsilon': 4.0},
            {'icp_max_iters': 100},
        ]
    )
    ld.add_action(poses_estimator)

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    ld.add_action(rviz)
    
    return ld
