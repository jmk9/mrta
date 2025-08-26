#!/usr/bin/env python3
from __future__ import annotations
from typing import Optional, List, Tuple

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from mrta_interfaces.srv import ReserveCharge, ReleaseCharge


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def make_pose_stamped(frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.0
    msg.pose.orientation = yaw_to_quaternion(yaw)
    return msg


class ExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__('mrta_executor')

        self.declare_parameter('robot_namespace', '')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('goal_xyyaw', [0.0, 0.0, 0.0])
        self.declare_parameter('use_charge_manager', True)

        self.robot_ns: str = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self.goal_frame: str = self.get_parameter('goal_frame').get_parameter_value().string_value
        g = list(self.get_parameter('goal_xyyaw').get_parameter_value().double_array_value)
        while len(g) < 3:
            g.append(0.0)
        self.goal: Tuple[float, float, float] = (g[0], g[1], g[2])
        self.use_charge_manager: bool = self.get_parameter('use_charge_manager').get_parameter_value().bool_value

        action_name = f'{self.robot_ns}/navigate_to_pose' if self.robot_ns else 'navigate_to_pose'
        self.nav_client: ActionClient = ActionClient(self, NavigateToPose, action_name)

        self.reserve_cli = self.create_client(ReserveCharge, 'reserve_charge')
        self.release_cli = self.create_client(ReleaseCharge, 'release_charge')

        self._reserved_station: Optional[str] = None
        self._started: bool = False
        self.create_timer(0.5, self._start_once)

    def _start_once(self) -> None:
        if self._started:
            return
        self._started = True
        self.get_logger().info(f'Executor start: ns="{self.robot_ns}", goal={self.goal}')
        self.execute_once()

    def execute_once(self) -> None:
        if self.use_charge_manager:
            if self.reserve_cli.wait_for_service(timeout_sec=3.0):
                req = ReserveCharge.Request()
                # TODO: srv 정의에 맞게 필드 세팅 (예: req.robot_id, req.soc 등)
                future = self.reserve_cli.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.done() and future.result() is not None:
                    resp = future.result()
                    accepted = getattr(resp, 'accepted', True)
                    self._reserved_station = getattr(resp, 'station_id', 'station_0') if accepted else None
                    self.get_logger().info(f'Reserve accepted={accepted}, station={self._reserved_station}')
                else:
                    self.get_logger().warn('Reserve failed or no response.')
            else:
                self.get_logger().warn('reserve_charge not available, skip.')

        if not self._wait_action_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not ready.')
            self._release_if_needed()
            return

        x, y, yaw = self.goal
        goal = NavigateToPose.Goal()
        goal.pose = make_pose_stamped(self.goal_frame, x, y, yaw)

        send_future = self.nav_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Goal rejected or failed to send.')
            self._release_if_needed()
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        status: int = getattr(result, 'status', -1) if result else -1
        self.get_logger().info(f'NavigateToPose finished with status={status}')

        self._release_if_needed()

    def _on_feedback(self, feedback: NavigateToPose.Feedback) -> None:
        # 필요 시 진행률 로깅
        pass

    def _wait_action_server(self, timeout_sec: float) -> bool:
        waited = 0.0
        step = 0.2
        while waited < timeout_sec and not self.nav_client.server_is_ready():
            rclpy.spin_once(self, timeout_sec=step)
            waited += step
        return self.nav_client.server_is_ready()

    def _release_if_needed(self) -> None:
        if not self.use_charge_manager or not self._reserved_station:
            return
        if self.release_cli.wait_for_service(timeout_sec=3.0):
            req = ReleaseCharge.Request()
            if hasattr(req, 'station_id'):
                setattr(req, 'station_id', self._reserved_station)
            future = self.release_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            self.get_logger().info('Release requested.')
        else:
            self.get_logger().warn('release_charge not available.')
        self._reserved_station = None


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
