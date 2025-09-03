# mrta_core/energy_model.py
from __future__ import annotations
from dataclasses import dataclass
from typing import Tuple
import math

@dataclass
class WheelPowerParams:
    # 논문 계수
    sigma0: float = 0.0
    sigma1: float = 0.0
    sigma2: float = 0.0
    sigma3: float = 0.0
    p_standby: float = 0.0
    # 로봇 파라미터
    wheel_radius: float = 0.033   # [m] TB3 예시
    axle_track: float = 0.160     # [m] 바퀴간 거리 (L)
    mass: float = 3.3             # [kg] 탑재 포함 질량
    n_wheels: int = 2             # 미분구동: 2

class EnergyModel:
    """
    기존 근사 모델 + 논문식(휠 각속도 기반) 물리 모델 모두 지원.
    """
    def __init__(self, params: WheelPowerParams | None = None) -> None:
        self.params = params or WheelPowerParams()

    # --- 기존 간단 모델 (유지) ---
    def estimate_time_energy(self, d_lin: float, d_ang: float,
                             v_max: float, w_max: float) -> Tuple[float, float]:
        """
        단순 근사: 직선/회전 별 최대속도 주행 가정.
        E = P_avg * T 형태(여기선 보수적으로 p_standby 사용).
        """
        v = max(1e-6, min(v_max,  v_max))
        w = max(1e-6, min(w_max,  w_max))
        t_lin = abs(d_lin) / v
        t_ang = abs(d_ang) / w
        T = t_lin + t_ang
        # 최소한 대기전력만 잡아도 과소추정보단 안전
        E_Wh = (self.params.p_standby * T) / 3600.0
        return T, E_Wh

    # --- 논문식 물리 모델 ---
    def estimate_time_energy_physical(self, d_lin: float, d_ang: float,
                                      v_max: float, w_max: float) -> Tuple[float, float]:
        """
        논문식 p_act를 사용. 직선( w=0 ) + 제자리회전( v=0 ) 두 세그먼트 합산.
        각 세그먼트에서 바퀴 각속도는 상수이므로 p_act 상수 -> E = p * T.
        """
        p = self.params
        # 세그먼트 1: 직선 주행
        v = max(1e-6, min(v_max, v_max))
        t_lin = abs(d_lin) / v
        # wheel omegas (w = 0)
        omega_r_lin = v / p.wheel_radius
        omega_l_lin = v / p.wheel_radius

        p_lin = self._p_act(omega_l_lin, omega_r_lin)

        # 세그먼트 2: 제자리 회전
        w = max(1e-6, min(w_max, w_max))
        t_ang = abs(d_ang) / w
        # v = 0, w ≠ 0  →  ω_R = (L/2 * w)/R, ω_L = -(L/2 * w)/R
        omega_r_rot = (p.axle_track * 0.5 * w) / p.wheel_radius
        omega_l_rot = -omega_r_rot  # 반대 방향
        # 회전 방향 부호 반영 (sgn(ω))
        if d_ang < 0.0:
            omega_r_rot *= -1.0
            omega_l_rot *= -1.0

        p_rot = self._p_act(omega_l_rot, omega_r_rot)

        T = t_lin + t_ang
        E_Wh = (p_lin * t_lin + p_rot * t_ang) / 3600.0  # J/s * s -> J ; 1Wh = 3600J 가정
        return T, E_Wh

    def _p_act(self, omega_l: float, omega_r: float) -> float:
        """
        p_act = p_standby + Σ_i (σ0 + σ1 ω_i^2 + σ2 sgn(ω_i) ω_i) + σ3 m
        """
        p = self.params
        def wheel_term(omega: float) -> float:
            return p.sigma0 + p.sigma1 * (omega ** 2) + p.sigma2 * math.copysign(1.0, omega) * abs(omega)

        # N=2(미분구동) 일반화: 필요 시 n_wheels>2로 확장 가능
        s = wheel_term(omega_l) + wheel_term(omega_r)
        return p.p_standby + s + p.sigma3 * p.mass
