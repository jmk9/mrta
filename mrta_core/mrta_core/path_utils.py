from typing import Tuple, List
import math
from geometry_msgs.msg import PoseStamped

def path_length_from_poses(poses: List[PoseStamped]) -> Tuple[float, float]:
    if not poses or len(poses) < 2:
        return 0.0, 0.0
    d_lin, d_ang = 0.0, 0.0
    prev = poses[0].pose
    prev_yaw = _yaw(prev)
    for ps in poses[1:]:
        p = ps.pose
        dx = p.position.x - prev.position.x
        dy = p.position.y - prev.position.y
        d_lin += math.hypot(dx, dy)
        yaw = _yaw(p)
        d = (yaw - prev_yaw + math.pi) % (2 * math.pi) - math.pi
        d_ang += abs(d)
        prev, prev_yaw = p, yaw
    return d_lin, d_ang

def _yaw(pose) -> float:
    q = pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)
