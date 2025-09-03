from __future__ import annotations
from typing import Optional, Tuple, List
import math
import random
import rclpy
from rclpy.node import Node
from rclpy.task import Future
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import ComputePathToPose

from mrta_core.energy_model import EnergyModel
from mrta_core.path_utils import path_length_from_poses


class SchedulerNode(Node):
    def __init__(self) -> None:
        super().__init__('scheduler')

        # ------- Parameters -------
        self.declare_parameter('robot_ns', 'tb3_0')
        self.declare_parameter('planner_service_fmt', '/{ns}/planner_server/compute_path_to_pose')
        self.declare_parameter('v_max', 0.22)
        self.declare_parameter('w_max', 2.84)

        # ------- Energy model (논문식 포함) -------
        self.declare_parameter('use_physical_energy', True)
        self.declare_parameter('sigma0', 0.0)
        self.declare_parameter('sigma1', 0.0)
        self.declare_parameter('sigma2', 0.0)
        self.declare_parameter('sigma3', 0.0)
        self.declare_parameter('p_standby', 0.0)
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('axle_track', 0.160)
        self.declare_parameter('mass', 3.3)

        # APSO / objective params
        self.declare_parameter('alpha_time', 1.0)
        self.declare_parameter('beta_energy', 1.0)
        self.declare_parameter('search_xmin', -1.0)
        self.declare_parameter('search_xmax', 3.0)
        self.declare_parameter('search_ymin', -1.0)
        self.declare_parameter('search_ymax', 3.0)

        self.declare_parameter('apso_particles', 16)
        self.declare_parameter('apso_iters', 25)
        self.declare_parameter('inertia_w_max', 0.9)
        self.declare_parameter('inertia_w_min', 0.4)
        self.declare_parameter('c1_start', 2.5)
        self.declare_parameter('c1_end', 0.5)
        self.declare_parameter('c2_start', 0.5)
        self.declare_parameter('c2_end', 2.5)
        self.declare_parameter('vmax_frac', 0.25)

        # ------- Read params -------
        self._robot_ns: str = self.get_parameter('robot_ns').get_parameter_value().string_value
        self._svc: str = self.get_parameter('planner_service_fmt').get_parameter_value().string_value.format(ns=self._robot_ns)
        self._v_max: float = float(self.get_parameter('v_max').value)
        self._w_max: float = float(self.get_parameter('w_max').value)

        # Energy model params
        self._use_phys: bool = bool(self.get_parameter('use_physical_energy').value)
        self._sigma0: float = float(self.get_parameter('sigma0').value)
        self._sigma1: float = float(self.get_parameter('sigma1').value)
        self._sigma2: float = float(self.get_parameter('sigma2').value)
        self._sigma3: float = float(self.get_parameter('sigma3').value)
        self._p_stdby: float = float(self.get_parameter('p_standby').value)
        self._wheel_R: float = float(self.get_parameter('wheel_radius').value)
        self._axle_L: float = float(self.get_parameter('axle_track').value)
        self._mass: float = float(self.get_parameter('mass').value)

        self._alpha_time: float = float(self.get_parameter('alpha_time').value)
        self._beta_energy: float = float(self.get_parameter('beta_energy').value)

        self._xmin: float = float(self.get_parameter('search_xmin').value)
        self._xmax: float = float(self.get_parameter('search_xmax').value)
        self._ymin: float = float(self.get_parameter('search_ymin').value)
        self._ymax: float = float(self.get_parameter('search_ymax').value)

        self._n_particles: int = int(self.get_parameter('apso_particles').value)
        self._n_iters: int = int(self.get_parameter('apso_iters').value)
        self._w_max_par: float = float(self.get_parameter('inertia_w_max').value)
        self._w_min_par: float = float(self.get_parameter('inertia_w_min').value)
        self._c1_start: float = float(self.get_parameter('c1_start').value)
        self._c1_end: float = float(self.get_parameter('c1_end').value)
        self._c2_start: float = float(self.get_parameter('c2_start').value)
        self._c2_end: float = float(self.get_parameter('c2_end').value)
        self._vmax_frac: float = float(self.get_parameter('vmax_frac').value)

        # ------- ROS clients / helpers -------
        self._energy = EnergyModel()
        if hasattr(self._energy, "params") and self._energy.params is not None:
            self._energy.params.sigma0 = self._sigma0
            self._energy.params.sigma1 = self._sigma1
            self._energy.params.sigma2 = self._sigma2
            self._energy.params.sigma3 = self._sigma3
            self._energy.params.p_standby = self._p_stdby
            self._energy.params.wheel_radius = self._wheel_R
            self._energy.params.axle_track = self._axle_L
            self._energy.params.mass = self._mass

        self._cli = self.create_client(ComputePathToPose, self._svc)
        self._demo_timer = self.create_timer(1.0, self._demo_call_once)

    # =========================== DEMO ===========================
    def _demo_call_once(self) -> None:
        self._demo_timer.cancel()
        if not self._cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service not available: {self._svc}')
            return

        start = PoseStamped()
        start.header.frame_id = 'map'
        start.pose.orientation.w = 1.0

        best_xy, best_cost = self._apso_optimize(start)
        bx, by = best_xy

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = float(bx)
        goal.pose.position.y = float(by)
        goal.pose.orientation.w = 1.0

        req = ComputePathToPose.Request()
        req.start = start
        req.goal = goal
        try:
            req.use_start = True
        except AttributeError:
            pass

        fut: Future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        resp: Optional[ComputePathToPose.Response] = fut.result()

        if resp is None:
            dlin = math.hypot(goal.pose.position.x - start.pose.position.x,
                              goal.pose.position.y - start.pose.position.y)
            dang = 0.0
        else:
            poses = resp.path.poses if hasattr(resp, 'path') else None
            dlin, dang = path_length_from_poses(poses)

        if self._use_phys and hasattr(self._energy, "estimate_time_energy_physical"):
            T_sec, E_Wh = self._energy.estimate_time_energy_physical(dlin, dang, self._v_max, self._w_max)
        else:
            T_sec, E_Wh = self._energy.estimate_time_energy(dlin, dang, self._v_max, self._w_max)

        self.get_logger().info(
            f'[APSO] ns={self._robot_ns} best=({bx:.2f},{by:.2f}) '
            f'dlin={dlin:.3f}m dang={dang:.3f}rad -> T={T_sec:.2f}s E={E_Wh:.3f}Wh '
            f'cost={best_cost:.3f} (α={self._alpha_time}, β={self._beta_energy})'
        )

    # =========================== APSO CORE ===========================
    class _Particle:
        def __init__(self, x: float, y: float, vx: float, vy: float) -> None:
            self.x = x
            self.y = y
            self.vx = vx
            self.vy = vy
            self.best_x = x
            self.best_y = y
            self.best_cost = math.inf

    def _apso_optimize(self, start: PoseStamped) -> Tuple[Tuple[float, float], float]:
        rng = random.Random(42)
        xr = self._xmax - self._xmin
        yr = self._ymax - self._ymin
        vclamp_x = self._vmax_frac * xr
        vclamp_y = self._vmax_frac * yr

        swarm: List[SchedulerNode._Particle] = []
        for _ in range(self._n_particles):
            x0 = rng.uniform(self._xmin, self._xmax)
            y0 = rng.uniform(self._ymin, self._ymax)
            vx0 = rng.uniform(-vclamp_x, vclamp_x)
            vy0 = rng.uniform(-vclamp_y, vclamp_y)
            swarm.append(SchedulerNode._Particle(x0, y0, vx0, vy0))

        gbest_x, gbest_y, gbest_cost = math.nan, math.nan, math.inf

        for p in swarm:
            c = self._evaluate_cost(start, p.x, p.y)
            p.best_cost = c
            p.best_x, p.best_y = p.x, p.y
            if c < gbest_cost:
                gbest_cost, gbest_x, gbest_y = c, p.x, p.y

        for it in range(1, self._n_iters + 1):
            frac = it / float(self._n_iters)
            w = self._w_max_par - (self._w_max_par - self._w_min_par) * frac
            c1 = self._c1_start - (self._c1_start - self._c1_end) * frac
            c2 = self._c2_start + (self._c2_end - self._c2_start) * frac

            for p in swarm:
                r1, r2 = rng.random(), rng.random()
                p.vx = w * p.vx + c1 * r1 * (p.best_x - p.x) + c2 * r2 * (gbest_x - p.x)
                p.vy = w * p.vy + c1 * r1 * (p.best_y - p.y) + c2 * r2 * (gbest_y - p.y)
                p.vx = max(-vclamp_x, min(vclamp_x, p.vx))
                p.vy = max(-vclamp_y, min(vclamp_y, p.vy))
                p.x += p.vx
                p.y += p.vy

                if p.x < self._xmin or p.x > self._xmax:
                    p.vx *= -0.5
                    p.x = min(max(p.x, self._xmin), self._xmax)
                if p.y < self._ymin or p.y > self._ymax:
                    p.vy *= -0.5
                    p.y = min(max(p.y, self._ymin), self._ymax)

                c = self._evaluate_cost(start, p.x, p.y)
                if c < p.best_cost:
                    p.best_cost, p.best_x, p.best_y = c, p.x, p.y
                if c < gbest_cost:
                    gbest_cost, gbest_x, gbest_y = c, p.x, p.y

        return (gbest_x, gbest_y), gbest_cost

    # =========================== OBJECTIVE ===========================
    def _evaluate_cost(self, start: PoseStamped, x: float, y: float) -> float:
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = 1.0

        req = ComputePathToPose.Request()
        req.start = start
        req.goal = goal
        try:
            req.use_start = True
        except AttributeError:
            pass

        fut: Future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.5)
        resp: Optional[ComputePathToPose.Response] = fut.result()

        if resp is None:
            dlin = math.hypot(goal.pose.position.x - start.pose.position.x,
                              goal.pose.position.y - start.pose.position.y)
            dang = 0.0
        else:
            poses = resp.path.poses if hasattr(resp, 'path') else None
            dlin, dang = path_length_from_poses(poses)

        if self._use_phys and hasattr(self._energy, "estimate_time_energy_physical"):
            T_sec, E_Wh = self._energy.estimate_time_energy_physical(dlin, dang, self._v_max, self._w_max)
        else:
            T_sec, E_Wh = self._energy.estimate_time_energy(dlin, dang, self._v_max, self._w_max)

        return self._alpha_time * T_sec + self._beta_energy * E_Wh


def main() -> None:
    rclpy.init()
    node = SchedulerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
