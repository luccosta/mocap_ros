from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ifopt_multirobot_planner',
            executable='planner_node',
            name='planner',
            output='screen',
            parameters=[{
                'dt': 0.05,
                'H': 40,
                'corridor_width': 0.8,
                'robot_radius': 0.25,
                'v_max': 0.8,
                'w_max': 1.5,
            }]
        )
    ])
