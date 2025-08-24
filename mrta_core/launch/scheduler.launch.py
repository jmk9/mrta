from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mrta_core',
            executable='scheduler_node',
            name='scheduler',
            output='screen',
            parameters=[{
                'robot_ns': 'tb3_0',
                'planner_service_fmt': '/{ns}/planner_server/compute_path_to_pose',
                'v_max': 0.22,
                'w_max': 2.84,
            }],
        ),
    ])
