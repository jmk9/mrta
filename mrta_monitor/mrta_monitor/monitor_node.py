#!/usr/bin/env python3
from __future__ import annotations
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from mrta_interfaces.msg import RobotState


class MonitorNode(Node):
    """
    Collects robot SoC, pose, etc., and republishes as RobotState messages.
    """
    def __init__(self) -> None:
        super().__init__('mrta_monitor')

        # robot namespace parameter
        self.declare_parameter('robot_namespace', '')
        self.robot_ns: str = self.get_parameter('robot_namespace').get_parameter_value().string_value

        # Subscriptions (예: /tb3_0/soc, /tb3_0/odom)
        soc_topic = f'{self.robot_ns}/soc' if self.robot_ns else 'soc'
        odom_topic = f'{self.robot_ns}/odom' if self.robot_ns else 'odom'

        self.soc_sub = self.create_subscription(Float32, soc_topic, self._on_soc, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        # Publisher: aggregated RobotState
        self.state_pub = self.create_publisher(RobotState, 'robot_states', 10)

        # Internal state
        self._soc: Optional[float] = None
        self._odom: Optional[Odometry] = None

        # Periodic publish
        self.create_timer(1.0, self._publish_state)

        self.get_logger().info(f'Monitor started for ns="{self.robot_ns}"')

    def _on_soc(self, msg: Float32) -> None:
        self._soc = msg.data

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _publish_state(self) -> None:
        if self._soc is None or self._odom is None:
            return
        state = RobotState()
        state.robot_id = self.robot_ns if self.robot_ns else 'robot'
        state.soc = self._soc
        state.pose = self._odom.pose.pose
        # TODO: state.current_task 등 다른 필드도 필요시 채우기
        self.state_pub.publish(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
