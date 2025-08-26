from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mrta_monitor',
            executable='monitor_node',
            name='monitor_tb3_0',
            namespace='/tb3_0',
            output='screen',
            parameters=[{'robot_namespace': '/tb3_0'}],
        ),
        Node(
            package='mrta_monitor',
            executable='monitor_node',
            name='monitor_tb3_1',
            namespace='/tb3_1',
            output='screen',
            parameters=[{'robot_namespace': '/tb3_1'}],
        ),
    ])
