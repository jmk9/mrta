from typing import Tuple

class EnergyModel:
    def __init__(self, ps_mean_w: float = 6.35, ps_std_w: float = 0.34) -> None:
        self.ps_mean_w = ps_mean_w
        self.ps_std_w = ps_std_w

    def static_power_w(self) -> float:
        return self.ps_mean_w  # 재현성 위해 고정

    def estimate_time_energy(
        self,
        dlin_m: float,
        dang_rad: float,
        v_ms: float,
        w_rads: float,
        payload_kg: float = 0.0,
    ) -> Tuple[float, float]:
        v = max(1e-3, v_ms)
        w = max(1e-3, w_rads)
        T = (dlin_m / v) + (dang_rad / w)
        Ps = self.static_power_w()
        Pact = 8.0 + 0.2 * v_ms + 0.05 * abs(w_rads) + 0.1 * payload_kg
        E_Wh = (Ps + Pact) * T / 3600.0
        return T, E_Wh
