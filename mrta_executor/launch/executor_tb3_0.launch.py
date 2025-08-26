from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mrta_executor',
            executable='executor_node',
            name='executor_tb3_0',
            namespace='/tb3_0',
            output='screen',
            parameters=[
                {'robot_namespace': '/tb3_0'},
                {'goal_frame': 'map'},
                {'goal_xyyaw': [1.0, 0.0, 0.0]},
                {'use_charge_manager': True},
            ],
        )
    ])
