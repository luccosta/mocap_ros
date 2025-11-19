from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from launch.substitutions import PathJoinSubstitution

gopigo_parameters = [
    {'velocity_gain': 0.5},
    {'orientation_gain': 1.5},
]

waverover_parameters = [
    {'velocity_gain': 1.0},
    {'orientation_gain': 3.0},
]

create_parameters = [
    {'velocity_gain': 0.5},
    {'orientation_gain': 1.5},
]


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

    wave_rover_driver = Node(
        package="wave_rover_ros_driver",
        executable='wave_rover_ros_driver',
        name='wave_rover_driver_node',
        output='screen',
        parameters=[
            {'robot_ip': '192.168.0.157'},
        ],
        remappings=[
            ('/cmd_vel', '/waverover1/cmd_vel')
        ],
    )
    ld.add_action(wave_rover_driver)

    wave_rover_driver = Node(
        package="wave_rover_ros_driver",
        executable='wave_rover_ros_driver',
        name='wave_rover_driver_node',
        output='screen',
        parameters=[
            {'robot_ip': '192.168.0.159'},
        ],
        remappings=[
            ('/cmd_vel', '/waverover2/cmd_vel')
        ],
    )
    ld.add_action(wave_rover_driver)

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

    go_to_goal = Node(
        package="go_to_goal",
        executable='go_to_goal',
        name='go_to_goal_node',
        output='screen',
        parameters=create_parameters,
        remappings=[
            ('/robot_pose', '/create3/icp_pose'),
            ('/target_pose', '/create3/target_pose'),
            ('/cmd_vel', '/create3/cmd_vel')
        ],
    )
    ld.add_action(go_to_goal)

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    ld.add_action(rviz)
    
    return ld
