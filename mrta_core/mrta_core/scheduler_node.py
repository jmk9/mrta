from typing import Optional
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import ComputePathToPose

from mrta_core.energy_model import EnergyModel
from mrta_core.path_utils import path_length_from_poses

class SchedulerNode(Node):
    def __init__(self) -> None:
        super().__init__('scheduler')
        self.declare_parameter('robot_ns', 'tb3_0')
        self.declare_parameter('planner_service_fmt', '/{ns}/planner_server/compute_path_to_pose')
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('w_max', 2.84)

        self._robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        self._svc = self.get_parameter('planner_service_fmt').get_parameter_value().string_value.format(ns=self._robot_ns)
        self._v_max = float(self.get_parameter('v_max').value)
        self._w_max = float(self.get_parameter('w_max').value)

        self._energy = EnergyModel()
        self._cli = self.create_client(ComputePathToPose, self._svc)
        self._demo_timer = self.create_timer(1.0, self._demo_call_once)

    def _demo_call_once(self) -> None:
        self._demo_timer.cancel()
        if not self._cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service not available: {self._svc}')
            return

        start = PoseStamped()
        start.header.frame_id = 'map'
        start.pose.orientation.w = 1.0

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 2.0
        goal.pose.position.y = 1.0
        goal.pose.orientation.w = 1.0

        req = ComputePathToPose.Request()
        req.start = start
        req.goal = goal
        try:
            req.use_start = True  # Jazzy는 존재, 다른 배포판 대비
        except AttributeError:
            pass

        future = self._cli.call_async(req)
        future.add_done_callback(self._on_plan_response)

    def _on_plan_response(self, fut) -> None:
        resp: Optional[ComputePathToPose.Response] = fut.result()
        if resp is None:
            self.get_logger().error('ComputePathToPose: no response')
            return
        poses = resp.path.poses if hasattr(resp, 'path') else None
        dlin, dang = path_length_from_poses(poses)
        T_sec, E_Wh = self._energy.estimate_time_energy(dlin, dang, self._v_max, self._w_max)
        self.get_logger().info(
            f'ns={self._robot_ns} dlin={dlin:.3f}m dang={dang:.3f}rad -> T={T_sec:.2f}s E={E_Wh:.3f}Wh'
        )

def main() -> None:
    rclpy.init()
    node = SchedulerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
