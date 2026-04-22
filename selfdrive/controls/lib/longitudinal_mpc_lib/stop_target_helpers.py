import numpy as np

from openpilot.common.constants import CV

STOP_TARGET_LATCH_S = 0.6
_SPEED_BP_KPH = [0.0, 1.5, 3.5, 5.5, 6.0, 6.5, 7.5]
_SPEED_FACTOR = [1.0, 0.95, 0.78, 0.75, 0.65, 0.45, 0.0]
_MAX_DISTANCE_M = 4.5


def get_distance_to_stopped_lead_target(v_lead_raw, d_lead_raw) -> float:
  v_lead = float(np.mean(v_lead_raw))
  d_lead = float(np.mean(d_lead_raw))
  if d_lead <= 0.0 or d_lead > _MAX_DISTANCE_M:
    return 0.0
  factor = float(np.interp(v_lead * CV.MS_TO_KPH, _SPEED_BP_KPH, _SPEED_FACTOR))
  return max(0.0, d_lead * factor)


def update_distance_to_stop_target_with_latch(
  last: float,
  latch_s: float,
  dt: float,
  candidates: tuple[float, ...],
) -> tuple[float, float]:
  valid = [c for c in candidates if c > 0.0]
  if valid:
    return min(valid), STOP_TARGET_LATCH_S
  if last > 0.0 and latch_s > 0.0:
    return last, max(0.0, latch_s - dt)
  return -1.0, 0.0
