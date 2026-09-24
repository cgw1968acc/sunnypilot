"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.sunnypilot import get_sanitize_int_param

AccelProfile = custom.LongitudinalPlanSP.AccelController.Profile

# Breakpoints at 60/70/80 km/h were added for the Corolla Cross Hybrid (2026-09-25). Normal/sport hold their
# previous straight-line values at the new points; eco below is its previous line too (0.40 @ 43 km/h -> 0.15 @ 86),
# used whenever the engine is already running (or the car does not report engine state).
MAX_ACCEL_BREAKPOINTS = [0., 3., 12., 16.67, 19.44, 22.22, 24., 36.]  # m/s (16.67/19.44/22.22 = 60/70/80 km/h)
MAX_ACCEL_PROFILES = {
  AccelProfile.eco:    [1.70, 1.38, 0.40, 0.30, 0.245, 0.19, 0.15, 0.10],
  AccelProfile.normal: [1.80, 1.55, 0.55, 0.48, 0.43, 0.39, 0.36, 0.25],
  AccelProfile.sport:  [2.00, 2.00, 1.20, 1.00, 0.89, 0.78, 0.70, 0.50],
}
# Eco while the hybrid's engine is OFF: keep wheel power m*a*v + road load (m=1500 kg, Crr 0.010, CdA 0.82) under the
# ~11-14 kW at which the Corolla Cross starts its engine (30 rlogs, 122 accel-driven starts, SOC 35-60%; the threshold
# does not move with SOC, rises mildly with speed: p10 14 kW at 80-89 km/h). 60 km/h 0.25, 70 km/h 0.16, 80 km/h 0.12.
# Once the engine runs there is nothing to save, so the normal eco line above takes over until it stops again.
ECO_ENGINE_OFF_MAX_ACCEL = [1.70, 1.38, 0.40, 0.25, 0.16, 0.12, 0.08, 0.06]
CRUISE_DECEL_RESPONSE_TIME = {  # seconds
  AccelProfile.eco: 4.0,
  AccelProfile.normal: 3.5,
  AccelProfile.sport: 3.0,
}
CRUISE_DECEL_ACCEL = {  # m/s^2; comfort-first cruise deceleration target
  AccelProfile.eco: -0.35,
  AccelProfile.normal: -0.50,
  AccelProfile.sport: -0.65,
}


class AccelController:
  def __init__(self):
    self.params = Params()
    self.update()

  def update(self) -> None:
    self._profile = get_sanitize_int_param("AccelPersonality", AccelProfile.eco, AccelProfile.sport, self.params)
    self._enabled = self.params.get_bool("AccelPersonalityEnabled")

  @property
  def profile(self) -> int:
    return self._profile

  def is_enabled(self) -> bool:
    return self._enabled

  def get_max_accel(self, v_ego: float, engine_off: bool = False) -> float:
    profile = ECO_ENGINE_OFF_MAX_ACCEL if (engine_off and self._profile == AccelProfile.eco) else MAX_ACCEL_PROFILES[self._profile]
    return float(np.interp(max(0.0, v_ego), MAX_ACCEL_BREAKPOINTS, profile))

  def get_cruise_target(self, v_ego: float, v_target: float) -> float:
    if not np.isfinite(v_target) or v_target <= 0.0 or v_target >= v_ego:
      return v_target

    response_time = CRUISE_DECEL_RESPONSE_TIME[self._profile]
    target_delta = v_target - v_ego
    if target_delta < CRUISE_DECEL_ACCEL[self._profile] * response_time * 2.0:
      return v_target
    return float(v_ego + target_delta / response_time)
