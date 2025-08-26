from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mrta_charge_manager',
            executable='charge_manager_node',
            name='charge_manager',
            output='screen',
            parameters=[{'stations_config': '$(find-pkg-share mrta_charge_manager)/config/stations.yaml'}],
        )
    ])
