def harvest_energy_wh(avg_harvest_w: float, dwell_sec: float) -> float:
    return max(0.0, avg_harvest_w) * max(0.0, dwell_sec) / 3600.0
